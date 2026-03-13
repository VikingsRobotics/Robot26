#ifndef SHOOTER_FLYWHEEL_H
#define SHOOTER_FLYWHEEL_H
#pragma once

#include "shooter/ShooterFlywheelConstants.hpp"

#include <units/velocity.h>
#include <units/current.h>
#include <units/voltage.h>
#include <units/time.h>

#include <rev/SparkFlex.h>

#include <frc/filter/LinearFilter.h>

#include <frc2/command/SubsystemBase.h>

#include <fmt/core.h>

#include <cmath>
#include <mutex>
#include <atomic>
#include <span>
#include <thread>

class ShooterRequest;

/**
 * \brief Subsystem representing a shooter flywheel mechanism.
 *
 * ShooterFlywheel manages a single flywheel motor and its associated
 * encoder and closed-loop controller. The subsystem provides a flexible
 * request-based control interface that allows external code to supply
 * control behaviors (ShooterRequest objects or lambdas) which are executed
 * during the control loop.
 *
 * The subsystem maintains a cached ShooterState containing timing and
 * diagnostic information about the control loop. This state can be queried
 * at any time using GetState().
 *
 * Control requests are applied through SetControl(), which stores a function
 * that will be executed periodically by the subsystem's control loop. The
 * function receives ControlParameters describing timing information and
 * access to the Flywheel hardware abstraction.
 */
class ShooterFlywheel : public frc2::SubsystemBase {
public:
    /**
     * \brief Plain-Old-Data class holding the state of the shooter flywheel.
     *
     * Contains measured data for telemetry, logging, and decision-making.
     */
    struct ShooterState {
        /** \brief The current measured flywheel velocity */
        units::meters_per_second_t Velocity{};

        /** \brief The target flywheel velocity */
        units::meters_per_second_t TargetVelocity{};

        /** \brief The applied motor output [-1,1] */
        units::dimensionless::scalar_t AppliedOutput{};

        /** \brief The measured motor output voltage */
        units::volt_t Voltage{};

        /** \brief The measured motor output current */
        units::ampere_t Current{};

        /** \brief Timestamp of the state capture in the timebase of utils#GetCurrentTime() */
        units::second_t Timestamp{};

        /** \brief Measured control update period */
        units::second_t OdometryPeriod{};

        /** \brief Number of successful data acquisitions */
        int32_t SuccessfulDaqs{};

        /** \brief Number of failed data acquisitions */
        int32_t FailedDaqs{};
    };

    /**
     * \brief Contains all parameters needed by shooter requests to calculate
     * outputs and predict shot behavior.
     */
    struct ControlParameters {
        /** \brief Gear ratio between the flywheel motor and the wheel */
        units::dimensionless::scalar_t kGearRatio;

        /** \brief Maximum flywheel surface speed at 12V (m/s) */
        units::meters_per_second_t kMaxSurfaceSpeed;

        /** \brief Flywheel radius (meters), used for tangential speed / energy calculations */
        units::meter_t kWheelRadius;

        /** \brief Flywheel mass (kg), used for energy/force calculations */
        units::kilogram_t kWheelMass;

        /** \brief Shooter pose estimated on the field */
        frc::Pose3d shooterPose;

        /** \brief Current flywheel linear surface velocity (m/s) */
        units::meters_per_second_t surfaceVelocity;

        /** \brief Timestamp of the current control application */
        units::second_t timestamp;

        /** \brief Update period of the control application */
        units::second_t updatePeriod;
    };

    struct Flywheel {
        rev::spark::SparkFlex& motor;
        rev::spark::SparkClosedLoopController& controller;
        units::meters_per_second_t desired{-1};
    };

    /** \brief Function type used to apply control to the flywheel */
    using ShooterRequestFunc = std::function<void(ControlParameters const&, Flywheel&)>;

private:
    rev::spark::SparkFlex flywheelMotor;
    rev::spark::SparkRelativeEncoder flywheelEncoder;
    rev::spark::SparkClosedLoopController flywheelController;

    mutable Flywheel flywheelMotorItems;

    const units::dimensionless::scalar_t kGearRatio;
    const units::meters_per_second_t kWheelMaxSpeed;
    const units::meter_t kWheelRadius;
    const units::kilogram_t kWheelMass;
    const frc::Transform3d kShooterLocation;

    frc::Pose3d cachedRobotPose{};

    ShooterRequestFunc requestToApply = [](ControlParameters const&, Flywheel&) {};
    ControlParameters requestParameters{};

    mutable std::recursive_mutex stateLock{};
    ShooterState cachedState{};

    /** \brief Function called whenever the shooter state is updated */
    std::function<void(ShooterState const&)> telemetryFunction = [](ShooterState const&) {};

    frc::LinearFilter<units::second_t> loopFilter;
    units::second_t lastTimestamp;

public:
    ShooterFlywheel(ShooterFlywheelConstants const& flywheelConstants);

    void Periodic() override;

    /**
     * \brief Applies the specified control request to this shooter flywheel.
     *
     * This captures the request by reference, so the request object must live
     * at least as long as the flywheel subsystem. Typically this means storing
     * the request as a member variable of the robot or subsystem.
     *
     * \param newRequest Request to apply
     */
    template <std::derived_from<ShooterRequest> Request>
        requires(!std::is_const_v<Request>)
    void SetControl(Request& newRequest) {
        SetControl([request = newRequest](auto const& params, auto modules) mutable { return request.Apply(params, modules); });
    }

    /**
     * \brief Applies the specified control request to this shooter flywheel.
     *
     * This overload moves the request object into the control function.
     *
     * \param newRequest Request to apply
     */
    template <std::derived_from<ShooterRequest> Request>
        requires(!std::is_const_v<Request>)
    void SetControl(Request&& newRequest) {
        SetControl([request = std::move(newRequest)](auto const& params, auto modules) mutable { return request.Apply(params, modules); });
    }

    /**
     * \brief Applies the specified control function to the flywheel.
     *
     * The provided function will be stored and executed during the control loop.
     *
     * \param request Control function to apply
     */
    void SetControl(ShooterRequestFunc&& request) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};

        if (request) {
            requestToApply = std::move(request);
        } else {
            requestToApply = [](auto&, auto&) {};
        }
    }

    /**
     * \brief Updates the estimated shooter position.
     *
     * Updates the cached robot pose. When updating the robot pose, the
     * transformation of the shooter pose is update for requests runinng
     * on the shooter.
     *
     * \param robotLocation Position of the robot
     */
    void UpdateShooterPosition(frc::Pose3d robotLocation) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        cachedRobotPose = robotLocation;
    }

    /**
     * \brief Immediately runs a temporary control function.
     *
     * This bypasses the stored request and executes the provided control
     * function directly. It is intended for internal use (e.g., accelerated
     * control updates) and should only be called from the odometry/control thread.
     *
     * Otherwise, SetControl() should be used.
     *
     * \param request Temporary control function to invoke
     */
    void RunTempRequest(ShooterRequestFunc&& request) const {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        request(requestParameters, flywheelMotorItems);
    }

    /**
     * \brief Gets the transform of the shooter flywheel subsystem.
     *
     * This transform is from the robot center. To get the location in the field,
     * use this transform to get the Pose3d of the shooter flywheel by using
     * drivetrain estimated position.
     *
     * \returns Transform of shooter flywheel
     */
    frc::Transform3d GetTransform() const { return kShooterLocation; }

    /**
     * \brief Gets the current state of the shooter flywheel subsystem.
     *
     * This may include timing and other state data collected during
     * the odometry/control loop.
     *
     * \returns Current shooter state snapshot
     */
    ShooterState GetState() const {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        return cachedState;
    }

    /**
     * \brief Registers a telemetry callback executed whenever ShooterState is updated.
     *
     * This function is executed synchronously with the odometry/control loop,
     * so it **must remain lightweight** to avoid degrading system performance.
     *
     * Typical uses include:
     *  - Sending telemetry to dashboards
     *  - Logging diagnostic data
     *  - Copying the state for later processing
     *
     * \param func Telemetry or logging function
     */
    void RegisterTelemetry(std::function<void(ShooterState const&)> func) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        telemetryFunction = std::move(func);
    }
};

#endif