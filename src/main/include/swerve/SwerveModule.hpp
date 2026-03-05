#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H
#pragma once

#include <units/angle.h>
#include <units/velocity.h>

#include <rev/AbsoluteEncoder.h>
#include <rev/SparkMax.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <frc/geometry/Translation2d.h>

#include <frc/filter/LinearFilter.h>

#include "swerve/SwerveModuleConstants.hpp"

enum class ModuleSteerRequestType {
    SmartMotion = 0,
    Position = 1,
};

enum class ModuleDriveRequestType {
    OpenLoopVoltage = 0,
    Velocity = 1,
};

class SwerveModule {
public:
    /**
     * \brief Contains everything the swerve module needs to apply a request.
     */
    struct ModuleRequest {
        /**
         * \brief Unoptimized speed and direction the module should target.
         */
        frc::SwerveModuleState State{};
        /**
         * \brief Robot-centric wheel force feedforward to apply in the
         * X direction. X is defined as forward according to WPILib
         * convention, so this determines the forward force to apply.
         *
         * This force should include friction applied to the ground.
         */
        units::newton_t WheelForceFeedforwardX = 0_N;
        /**
         * \brief Robot-centric wheel force feedforward to apply in the
         * Y direction. Y is defined as to the left according to WPILib
         * convention, so this determines the force to apply to the left.
         *
         * This force should include friction applied to the ground.
         */
        units::newton_t WheelForceFeedforwardY = 0_N;
        /**
         * \brief The type of control request to use for the drive motor.
         */
        ModuleDriveRequestType DriveRequest = ModuleDriveRequestType::Velocity;
        /**
         * \brief The type of control request to use for the steer motor.
         */
        ModuleSteerRequestType SteerRequest = ModuleSteerRequestType::Position;
        /**
         * \brief The update period of the module request. Setting this to
         * a non-zero value adds a velocity feedforward to the steer motor.
         */
        units::second_t UpdatePeriod = 0_s;

        /**
         * \brief Modifies the State parameter and returns itself.
         *
         * Unoptimized speed and direction the module should target.
         *
         * \param newState Parameter to modify
         * \returns Itself
         */
        ModuleRequest& WithState(frc::SwerveModuleState newState) {
            this->State = std::move(newState);
            return *this;
        }

        /**
         * \brief Modifies the WheelForceFeedforwardX parameter and returns itself.
         *
         * Robot-centric wheel force feedforward to apply in the
         * X direction. X is defined as forward according to WPILib
         * convention, so this determines the forward force to apply.
         *
         * This force should include friction.
         *
         * \param newWheelForceFeedforwardX Parameter to modify
         * \returns Itself
         */
        ModuleRequest& WithWheelForceFeedforwardX(units::newton_t newWheelForceFeedforwardX) {
            this->WheelForceFeedforwardX = newWheelForceFeedforwardX;
            return *this;
        }

        /**
         * \brief Modifies the WheelForceFeedforwardY parameter and returns itself.
         *
         * Robot-centric wheel force feedforward to apply in the
         * Y direction. Y is defined as to the left according to WPILib
         * convention, so this determines the force to apply to the left.
         *
         * This force should include friction.
         *
         * \param newWheelForceFeedforwardY Parameter to modify
         * \returns Itself
         */
        ModuleRequest& WithWheelForceFeedforwardY(units::newton_t newWheelForceFeedforwardY) {
            this->WheelForceFeedforwardY = newWheelForceFeedforwardY;
            return *this;
        }

        /**
         * \brief Modifies the DriveRequest parameter and returns itself.
         *
         * The type of control request to use for the drive motor.
         *
         * \param newDriveRequest Parameter to modify
         * \returns Itself
         */
        ModuleRequest& WithDriveRequest(ModuleDriveRequestType newDriveRequest) {
            this->DriveRequest = newDriveRequest;
            return *this;
        }
        /**
         * \brief Modifies the SteerRequest parameter and returns itself.
         *
         * The type of control request to use for the steer motor.
         *
         * \param newSteerRequest Parameter to modify
         * \returns Itself
         */
        ModuleRequest& WithSteerRequest(ModuleSteerRequestType newSteerRequest) {
            this->SteerRequest = newSteerRequest;
            return *this;
        }

        /**
         * \brief Modifies the UpdatePeriod parameter and returns itself.
         *
         * The update period of the module request. Setting this to a
         * non-zero value adds a velocity feedforward to the steer motor.
         *
         * \param newUpdatePeriod Parameter to modify
         * \returns Itself
         */
        ModuleRequest& WithUpdatePeriod(units::second_t newUpdatePeriod) {
            this->UpdatePeriod = newUpdatePeriod;
            return *this;
        }
    };

    /**
     * \brief Contains everything the steer motor needs to apply a request.
     */
    struct SteerReq {
        /**
         * Different traits depending on use case
         *
         * VoltageOut: The amount of volts to use on steering motor
         * PositionVoltage: The feedforward of volts on position control
         */
        units::volt_t Feedforward;

        /**
         * Destionation that is desired for output of the steering motor
         *
         * Unused during VoltageOut
         */
        units::turn_t Position;

        enum class OutputType { VoltageOut, PositionVoltage } Output;

        /**
         * \brief Modifies the parameters and returns itself.
         *
         * The voltage of the module request.
         *
         * \param OutputLevel Parameter to modify
         * \returns Itself
         */
        SteerReq& WithVoltageOut(units::volt_t OutputLevel) {
            Feedforward = OutputLevel;
            Output = OutputType::VoltageOut;
            return *this;
        };

        /**
         * \brief Modifies the parameters and returns itself.
         *
         * The position of the module request along with voltage feedforwards.
         *
         * \param TargetPosition Parameter to modify
         * \param FeedforwardLevel Parameter to modify
         * \returns Itself
         */
        SteerReq& WithPositionVoltage(units::turn_t TargetPosition, units::volt_t FeedforwardLevel) {
            Position = TargetPosition;
            Feedforward = FeedforwardLevel;
            Output = OutputType::PositionVoltage;
            return *this;
        };
    };

private:
    ctre::phoenix6::hardware::TalonFX driveMotor;
    rev::spark::SparkMax steerMotor;

    rev::spark::SparkAbsoluteEncoder steerAbsoluteEncoder;
    rev::spark::SparkClosedLoopController& steerLoopController;

    units::turn_t chassisAngularOffset;
    frc::Translation2d moduleOffset;

    ctre::phoenix6::StatusSignal<units::turn_t> drivePosition;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocity;
    ctre::phoenix6::StatusSignal<units::turns_per_second_squared_t> driveAcceleration;

    mutable ctre::phoenix6::StatusSignal<ctre::unit::newton_meters_per_ampere_t> driveMotorKT;
    mutable ctre::phoenix6::StatusSignal<units::ampere_t> driveMotorStallCurrent;

    ctre::phoenix6::StatusSignal<units::ampere_t> driveMotorOutputCurrent;
    ctre::phoenix6::StatusSignal<units::volt_t> driveMotorOutputVoltage;

    mutable frc::LinearFilter<units::turns_per_second_t> moduleSupplem;

    using turns_per_meter = units::compound_unit<units::turns, units::inverse<units::meters>>;
    using turns_per_meter_t = units::unit_t<turns_per_meter>;

    turns_per_meter_t kDriveRotationsPerMeter;
    units::meter_t kDriveNmPerWheelN;
    units::scalar_t kCouplingRatioDriveRotorToEncoder;
    units::meters_per_second_t kSpeedAt12Volts;

    frc::SwerveModuleState targetState;

public:
    struct CacheState {
        struct Drive {
            units::meter_t position;
            units::meters_per_second_t velocity;
            units::meters_per_second_squared_t acceleration;
            units::volt_t voltage;
            units::ampere_t current;
        } drive;
        struct Azimuth {
            units::turn_t position;
            units::turns_per_second_t velocity;
            units::volt_t voltage;
            units::ampere_t current;
        } azimuth;
    };

private:
    CacheState cachedState;

public:
    /**
     * \brief Construct a SwerveModuleImpl with the specified constants.
     *
     * \param constants Constants used to construct the module
     * \param canbus    The CAN bus this module is on
     */
    SwerveModule(const SwerveModuleConstants& constants, ctre::phoenix6::CANBus canBus);

    /**
     * \brief Applies the desired ModuleRequest to this module.
     *
     * \param moduleRequest The request to apply to this module
     */
    void Apply(ModuleRequest const& moduleRequest);

    /**
     * \brief Controls this module using the specified drive and steer control requests.
     *
     * This is intended only to be used for characterization of the robot; do not use this for normal use.
     *
     * \param driveRequest The control request to apply to the drive motor
     * \param steerRequest The control request to apply to the steer motor
     */
    template <std::derived_from<ctre::phoenix6::controls::ControlRequest> DriveReq>
    void Apply(DriveReq&& driveRequest, SteerReq&& steerRequest) {
        driveMotor.SetControl(driveRequest.WithUpdateFreqHz(0_Hz));

        switch (steerRequest.Output) {
            case SteerReq::OutputType::VoltageOut:
                steerMotor.SetVoltage(steerRequest.Feedforward);
                break;

            case SteerReq::OutputType::PositionVoltage:
                steerLoopController.SetSetpoint(steerRequest.Position(), rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::ClosedLoopSlot::kSlot0,
                                                steerRequest.Feedforward(), rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
                break;

            default:
                steerMotor.StopMotor();
                break;
        }
    }

    /**
     * \brief Gets the state of this module and passes it back as a
     * SwerveModulePosition object.
     *
     * This function is blocking when it performs a refresh.
     *
     * \param refresh True if the signals should be refreshed
     * \returns SwerveModulePosition containing this module's state.
     */
    frc::SwerveModulePosition GetPosition(bool refresh);

    /**
     * \brief Get the current state of the module.
     *
     * This is typically used for telemetry, as the SwerveModulePosition
     * is used for odometry.
     *
     * \returns Current state of the module
     */
    frc::SwerveModuleState GetCurrentState() const {
        return frc::SwerveModuleState{driveVelocity.GetValue() / kDriveRotationsPerMeter, units::turn_t{steerAbsoluteEncoder.GetPosition()}};
    }

    /**
     * \brief Get the target state of the module.
     *
     * This is typically used for telemetry.
     *
     * \returns Target state of the module
     */
    frc::SwerveModuleState GetTargetState() const { return targetState; }

    /**
     * \brief Get the state of the module.
     *
     * This is typically used for telemetry.
     *
     * \returns State of the module
     */
    CacheState GetCachedState() const { return cachedState; }

    /**
     * \brief Resets this module's drive motor position to 0 rotations.
     */
    void ResetPosition() { driveMotor.SetPosition(0_tr); }

    /**
     * \brief Get the drive motor of the module.
     *
     * \returns Drive motor of the module
     */
    ctre::phoenix6::hardware::TalonFX& GetDriveMotor() { return driveMotor; }

    /**
     * \brief Get the azimuth motor of the module.
     *
     * \returns Azimuth motor of the module
     */
    rev::spark::SparkMax& GetAzimuthMotor() { return steerMotor; }

    /**
     * \brief Get the azimuth encoder of the module.
     *
     * \returns Azimuth absolute encoder of the module
     */
    rev::spark::SparkAbsoluteEncoder& GetAzimuthEncoder() { return steerAbsoluteEncoder; }

    /**
     * \brief Get the angular offset of the module.
     *
     * This can be used to find true zero of the module.
     *
     * \returns Angular offset of the module
     */
    units::turn_t GetAngularOffset() { return chassisAngularOffset; }

    /**
     * \brief Get the location offset of the module.
     *
     * This can be used by kinematics and odometry.
     *
     * \returns Location offset of the module
     */
    frc::Translation2d GetModuleLocation() { return moduleOffset; }

private:
    /**
     * \brief Collection of all possible torque feedforward outputs that
     * can be applied to the motor for a given wheel force feedforward.
     *
     * We only use voltage because we don't have PRO
     */
    struct MotorTorqueFeedforwards {
        units::newton_meter_t torque;
        units::ampere_t torqueCurrent;
        units::volt_t voltage;
    };

    units::turns_per_second_t ApplyVelocityCorrections(units::turns_per_second_t velocity, units::turn_t angleDifference) const;
    MotorTorqueFeedforwards CalculateMotorTorqueFeedforwards(units::newton_t wheelForceFeedforwardX, units::newton_t wheelForceFeedforwardY) const;

    friend class SwerveDrivetrain;
};

#endif
