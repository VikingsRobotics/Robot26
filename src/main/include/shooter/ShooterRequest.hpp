#ifndef SHOOTER_REQUEST_H
#define SHOOTER_REQUEST_H
#pragma once

#include "shooter/ShooterFlywheel.hpp"

#include <units/math.h>
#include <units/acceleration.h>
#include <units/velocity.h>

enum class FlywheelRequestType { kOpenLoop = 0, kClosedLoop = 1 };

class ShooterRequest {
public:
    using ControlParameters = ShooterFlywheel::ControlParameters;
    using Flywheel = ShooterFlywheel::Flywheel;

public:
    units::second_t startTimestamp{0};

public:
    virtual ~ShooterRequest() = default;

    /**
     * \brief Applies this swerve request to the given modules.
     * This is typically called by the SwerveDrivetrain.
     *
     * \param parameters Parameters the control request needs to calculate the module state
     * \param flywheel Motors to which the control request is applied
     * \returns Status code of sending the request
     */
    virtual units::meters_per_second_t Apply(ControlParameters const& parameters, Flywheel& flywheel) = 0;
};

/**
 * \brief Does nothing to the flywheel. This is the default state of a newly
 * created shooter flywheel.
 */
class Idle : public ShooterRequest {
public:
    units::meters_per_second_t Apply(ShooterRequest::ControlParameters const&, ShooterRequest::Flywheel&) override { return -1_mps; }
};

class ShooterBrake : public ShooterRequest {
public:
    units::meters_per_second_t Apply(ControlParameters const& parameters, Flywheel& flywheel) override {
        flywheel.motor.StopMotor();
        flywheel.feeder.StopMotor();
        return 0_mps;
    }
};

using meters_per_turn = units::compound_unit<units::meter, units::inverse<units::turns>>;
using meters_per_turn_t = units::unit_t<meters_per_turn>;

static constexpr meters_per_turn_t ConvertMetersToTurns(ShooterFlywheel::ControlParameters input) {
    return (input.kWheelRadius * 2 * std::numbers::pi) / (input.kGearRatio * 1_tr);
};


template <typename T, typename T1, typename T2>
static T Clamp(T value, T1 min, T2 max) {
    static_assert(units::traits::is_unit_t<T>::value, "Unit types are not compatible.");
    static_assert(units::traits::is_convertible_unit_t<T, T1>::value, "Unit types are not compatible.");
    static_assert(units::traits::is_convertible_unit_t<T, T2>::value, "Unit types are not compatible.");
    return units::math::min(max, units::math::max(min, value));
}

class RotationRequest : public ShooterRequest {
public:
    /**
     * \brief The speeds to apply to the flywheel.
     */
    units::turns_per_second_t Speeds{};
    /**
     * \brief The type of output to apply to the flywheel.
     */
    FlywheelRequestType Type = FlywheelRequestType::kClosedLoop;
    /**
     * \brief The speeds to apply to the feeder.
     */
    units::dimensionless::scalar_t FeederSpeed{};
    /**
     * \brief The when (after start) to apply to the feeder.
     */
    units::second_t FeederThreshold{};


    units::meters_per_second_t Apply(ControlParameters const& parameters, Flywheel& flywheel) override {
        units::meters_per_second_t desired =
            ConvertMetersToTurns(parameters) * Clamp(Speeds, 0_tps, (parameters.kMaxSurfaceSpeed / ConvertMetersToTurns(parameters)));
        if (Type == FlywheelRequestType::kOpenLoop) {
            flywheel.motor.SetVoltage((Speeds / (parameters.kMaxSurfaceSpeed / ConvertMetersToTurns(parameters))) * 12_V);
        } else {
            flywheel.controller.SetSetpoint(desired(), rev::spark::SparkLowLevel::ControlType::kVelocity);
        }

        if (parameters.timestamp - startTimestamp > FeederThreshold) {
            flywheel.feederControl.SetSetpoint(FeederSpeed(), rev::spark::SparkLowLevel::ControlType::kVelocity);
        } else {
            flywheel.feeder.StopMotor();
        }
        return desired;
    }

    /**
     * \brief Modifies the Speeds parameter and returns itself.
     *
     * The velocity in the flywheel in turns per second, so this determines how fast to travel forward.
     * Must be equal or greater than 0.
     *
     * \param newVelocity Parameter to modify
     * \returns this object
     */
    RotationRequest& WithSpeeds(units::turns_per_second_t newVelocity) & {
        this->Speeds = std::move(newVelocity);
        if (Speeds < 0_tps) {
            this->Speeds = 0_tps;
        }
        return *this;
    }
    /**
     * \brief Modifies the Speeds parameter and returns itself.
     *
     * The velocity in the flywheel in turns per second, so this determines how fast to travel forward.
     * Must be equal or greater than 0.
     *
     * \param newVelocity Parameter to modify
     * \returns this object
     */
    RotationRequest&& WithSpeeds(units::turns_per_second_t newVelocity) && {
        this->Speeds = std::move(newVelocity);
        if (Speeds < 0_tps) {
            this->Speeds = 0_tps;
        }
        return std::move(*this);
    }

    /**
     * \brief Modifies the Type parameter and returns itself.
     *
     * The flywheel operates with closed loop where feedback and feedforward is used.
     * The flywheel operates with open loop where estimated speeds are used from voltages.
     *
     * \param newType Parameter to modify
     * \returns this object
     */
    RotationRequest& WithType(FlywheelRequestType newType) & {
        this->Type = std::move(newType);
        return *this;
    }
    /**
     * \brief Modifies the Type parameter and returns itself.
     *
     * The flywheel operates with closed loop where feedback and feedforward is used.
     * The flywheel operates with open loop where estimated speeds are used from voltages.
     *
     * \param newType Parameter to modify
     * \returns this object
     */
    RotationRequest&& WithType(FlywheelRequestType newType) && {
        this->Type = std::move(newType);
        return std::move(*this);
    }

    /**
     * \brief Modifies the FeederSpeed parameter and returns itself.
     *
     * The feeder operates with closed loop where feedback is used.
     *
     * \param newSpeed Parameter to modify
     * \returns this object
     */
    RotationRequest& WithFeederSpeed(units::dimensionless::scalar_t newSpeed) & {
        this->FeederSpeed = std::move(newSpeed);
        return *this;
    }
    /**
     * \brief Modifies the FeederSpeed parameter and returns itself.
     *
     * The feeder operates with closed loop where feedback is used.
     *
     * \param newSpeed Parameter to modify
     * \returns this object
     */
    RotationRequest&& WithFeederSpeed(units::dimensionless::scalar_t newSpeed) && {
        this->FeederSpeed = std::move(newSpeed);
        return std::move(*this);
    }

    /**
     * \brief Modifies the FeederThreshold parameter and returns itself.
     *
     * The feeder operates after FeederThreshold time is passed after starting.
     * Use 0_s to activate right away.
     *
     * \param newThreshold Parameter to modify
     * \returns this object
     */
    RotationRequest& WithFeederTime(units::second_t newThreshold) & {
        this->FeederThreshold = std::move(newThreshold);
        return *this;
    }
    /**
     * \brief Modifies the FeederThreshold parameter and returns itself.
     *
     * The feeder operates after FeederThreshold time is passed after starting.
     * Use 0_s to activate right away.
     *
     * \param newThreshold Parameter to modify
     * \returns this object
     */
    RotationRequest&& WithFeederTime(units::second_t newThreshold) && {
        this->FeederThreshold = std::move(newThreshold);
        return std::move(*this);
    }
};

class SurfaceRequest : public ShooterRequest {
public:
    /**
     * \brief The speeds to apply to the flywheel.
     */
    units::meters_per_second_t Speeds{};
    /**
     * \brief The type of output to apply to the flywheel.
     */
    FlywheelRequestType Type = FlywheelRequestType::kClosedLoop;
    /**
     * \brief The speeds to apply to the feeder.
     */
    units::dimensionless::scalar_t FeederSpeed{};
    /**
     * \brief The when (after start) to apply to the feeder.
     */
    units::second_t FeederThreshold{};

    units::meters_per_second_t Apply(ControlParameters const& parameters, Flywheel& flywheel) override {
        units::meters_per_second_t desired = Clamp(Speeds, 0_mps, parameters.kMaxSurfaceSpeed);
        if (Type == FlywheelRequestType::kOpenLoop) {
            flywheel.motor.SetVoltage((Speeds / parameters.kMaxSurfaceSpeed) * 12_V);
        } else {
            flywheel.controller.SetSetpoint(desired(), rev::spark::SparkLowLevel::ControlType::kVelocity);
        }
        return desired;
    }
    /**
     * \brief Modifies the Speeds parameter and returns itself.
     *
     * The velocity in the flywheel in meters per second, so this determines how fast to travel forward.
     * Must be equal or greater than 0.
     *
     * \param newVelocity Parameter to modify
     * \returns this object
     */
    SurfaceRequest& WithSpeeds(units::meters_per_second_t newVelocity) & {
        this->Speeds = std::move(newVelocity);
        if (Speeds < 0_mps) {
            this->Speeds = 0_mps;
        }
        return *this;
    }
    /**
     * \brief Modifies the Speeds parameter and returns itself.
     *
     * The velocity in the flywheel in meters per second, so this determines how fast to travel forward.
     * Must be equal or greater than 0.
     *
     * \param newVelocity Parameter to modify
     * \returns this object
     */
    SurfaceRequest&& WithSpeeds(units::meters_per_second_t newVelocity) && {
        this->Speeds = std::move(newVelocity);
        if (Speeds < 0_mps) {
            this->Speeds = 0_mps;
        }
        return std::move(*this);
    }
    /**
     * \brief Modifies the Type parameter and returns itself.
     *
     * The flywheel operates with closed loop where feedback and feedforward is used.
     * The flywheel operates with open loop where estimated speeds are used from voltages.
     *
     * \param newType Parameter to modify
     * \returns this object
     */
    SurfaceRequest& WithType(FlywheelRequestType newType) & {
        this->Type = std::move(newType);
        return *this;
    }
    /**
     * \brief Modifies the Type parameter and returns itself.
     *
     * The flywheel operates with closed loop where feedback and feedforward is used.
     * The flywheel operates with open loop where estimated speeds are used from voltages.
     *
     * \param newType Parameter to modify
     * \returns this object
     */
    SurfaceRequest&& WithType(FlywheelRequestType newType) && {
        this->Type = std::move(newType);
        return std::move(*this);
    }

    /**
     * \brief Modifies the FeederSpeed parameter and returns itself.
     *
     * The feeder operates with closed loop where feedback is used.
     *
     * \param newSpeed Parameter to modify
     * \returns this object
     */
    SurfaceRequest& WithFeederSpeed(units::dimensionless::scalar_t newSpeed) & {
        this->FeederSpeed = std::move(newSpeed);
        return *this;
    }
    /**
     * \brief Modifies the FeederSpeed parameter and returns itself.
     *
     * The feeder operates with closed loop where feedback is used.
     *
     * \param newSpeed Parameter to modify
     * \returns this object
     */
    SurfaceRequest&& WithFeederSpeed(units::dimensionless::scalar_t newSpeed) && {
        this->FeederSpeed = std::move(newSpeed);
        return std::move(*this);
    }

    /**
     * \brief Modifies the FeederThreshold parameter and returns itself.
     *
     * The feeder operates after FeederThreshold time is passed after starting.
     * Use 0_s to activate right away.
     *
     * \param newThreshold Parameter to modify
     * \returns this object
     */
    SurfaceRequest& WithFeederTime(units::second_t newThreshold) & {
        this->FeederThreshold = std::move(newThreshold);
        return *this;
    }
    /**
     * \brief Modifies the FeederThreshold parameter and returns itself.
     *
     * The feeder operates after FeederThreshold time is passed after starting.
     * Use 0_s to activate right away.
     *
     * \param newThreshold Parameter to modify
     * \returns this object
     */
    SurfaceRequest&& WithFeederTime(units::second_t newThreshold) && {
        this->FeederThreshold = std::move(newThreshold);
        return std::move(*this);
    }
};
/**
 * \brief Calculates the flywheel surface velocity required to hit a target at a
 * specified horizontal distance and height.
 *
 * This request computes the launch velocity using the general projectile motion
 * equation that accounts for vertical displacement between the shooter and the
 * target.
 *
 * Unlike the simplified flat-ground equation:
 *
 *   range = (v^2 * sin(2θ)) / g
 *
 * this implementation solves the more general case where the target may be
 * above or below the shooter.
 *
 * Given:
 *
 *   d = horizontal distance to target
 *   h = vertical height difference (targetHeight - shooterHeight)
 *   θ = shooter launch angle
 *   g = gravitational acceleration
 *
 * The required launch velocity is:
 *
 *   v = sqrt( (g * d^2) / (2 * cos^2(θ) * (d * tan(θ) - h)) )
 *
 * Where:
 *
 *   d = targetDistance
 *   h = targetHeight - shooterPose.Z()
 *   θ = shooterPose.Rotation().Y() (pitch)
 *
 * The calculated velocity represents the required **linear surface velocity**
 * of the flywheel in meters per second.
 *
 * The request then delegates control to a SurfaceRequest which applies the
 * appropriate open-loop or closed-loop control mode.
 *
 * Assumptions:
 *
 *  - No air resistance (vacuum projectile model)
 *  - Ball exits the flywheel at the shooter pitch angle
 *  - Flywheel surface velocity approximately equals ball exit velocity
 *
 * Limitations:
 *
 *  - If (d * tan(θ) - h) ≤ 0, the shot is physically impossible for the
 *    current shooter angle and the request will not apply a command.
 *  - Real-world factors such as drag, wheel compression, and slip are not
 *    modeled and may require empirical tuning.
 *
 * This request is intended for autonomous shot calculation where the target
 * distance and height are known.
 */
class IdealDistanceRequest : public ShooterRequest {
public:
    /** Desired horizontal distance to the target */
    units::meter_t targetDistance;

    /** Target height above field */
    units::meter_t targetHeight;

    FlywheelRequestType Type = FlywheelRequestType::kClosedLoop;

    /**
     * \brief The speeds to apply to the feeder.
     */
    units::dimensionless::scalar_t FeederSpeed{};
    /**
     * \brief The when (after start) to apply to the feeder.
     */
    units::second_t FeederThreshold{};

    units::meters_per_second_t Apply(ControlParameters const& parameters, Flywheel& flywheel) override {
        units::radian_t theta = parameters.shooterPose.Rotation().Y();

        units::meter_t shooterHeight = parameters.shooterPose.Z();
        units::meter_t heightDelta = targetHeight - shooterHeight;

        units::dimensionless::scalar_t cosTheta = units::math::cos(theta);
        units::dimensionless::scalar_t tanTheta = units::math::tan(theta);

        units::meter_t denominator = 2.0 * cosTheta * cosTheta * (targetDistance * tanTheta - heightDelta);

        if (denominator <= 0_m) {
            return ShooterBrake{}.Apply(parameters, flywheel);  // impossible shot geometry
        }

        units::meters_per_second_t requiredVelocity = units::math::sqrt(units::standard_gravity_t{1} * targetDistance * targetDistance / denominator);

        return SurfaceRequest{}
            .WithSpeeds(requiredVelocity)
            .WithType(Type)
            .WithFeederSpeed(FeederSpeed)
            .WithFeederTime(FeederThreshold)
            .Apply(parameters, flywheel);
    }

    IdealDistanceRequest& WithDistance(units::meter_t distance) & {
        this->targetDistance = std::move(distance);
        return *this;
    }

    IdealDistanceRequest&& WithDistance(units::meter_t distance) && {
        this->targetDistance = std::move(distance);
        return std::move(*this);
    }

    IdealDistanceRequest& WithTargetHeight(units::meter_t height) & {
        this->targetHeight = std::move(height);
        return *this;
    }

    IdealDistanceRequest&& WithTargetHeight(units::meter_t height) && {
        this->targetHeight = std::move(height);
        return std::move(*this);
    }

    IdealDistanceRequest& WithType(FlywheelRequestType newType) & {
        this->Type = std::move(newType);
        return *this;
    }

    IdealDistanceRequest&& WithType(FlywheelRequestType newType) && {
        this->Type = std::move(newType);
        return std::move(*this);
    }

    /**
     * \brief Modifies the FeederSpeed parameter and returns itself.
     *
     * The feeder operates with closed loop where feedback is used.
     *
     * \param newSpeed Parameter to modify
     * \returns this object
     */
    IdealDistanceRequest& WithFeederSpeed(units::dimensionless::scalar_t newSpeed) & {
        this->FeederSpeed = std::move(newSpeed);
        return *this;
    }
    /**
     * \brief Modifies the FeederSpeed parameter and returns itself.
     *
     * The feeder operates with closed loop where feedback is used.
     *
     * \param newSpeed Parameter to modify
     * \returns this object
     */
    IdealDistanceRequest&& WithFeederSpeed(units::dimensionless::scalar_t newSpeed) && {
        this->FeederSpeed = std::move(newSpeed);
        return std::move(*this);
    }

    /**
     * \brief Modifies the FeederThreshold parameter and returns itself.
     *
     * The feeder operates after FeederThreshold time is passed after starting.
     * Use 0_s to activate right away.
     *
     * \param newThreshold Parameter to modify
     * \returns this object
     */
    IdealDistanceRequest& WithFeederTime(units::second_t newThreshold) & {
        this->FeederThreshold = std::move(newThreshold);
        return *this;
    }
    /**
     * \brief Modifies the FeederThreshold parameter and returns itself.
     *
     * The feeder operates after FeederThreshold time is passed after starting.
     * Use 0_s to activate right away.
     *
     * \param newThreshold Parameter to modify
     * \returns this object
     */
    IdealDistanceRequest&& WithFeederTime(units::second_t newThreshold) && {
        this->FeederThreshold = std::move(newThreshold);
        return std::move(*this);
    }
};

#endif