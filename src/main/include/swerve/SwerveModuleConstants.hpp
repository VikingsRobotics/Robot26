#ifndef SWERVE_MODULE_CONSTANT_H
#define SWERVE_MODULE_CONSTANT_H
#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/current.h>
#include <units/velocity.h>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <rev/config/ClosedLoopConfig.h>

struct SwerveModuleConstants {
    constexpr SwerveModuleConstants() = default;
    constexpr SwerveModuleConstants(const SwerveModuleConstants&) = default;
    /**
     * \brief CAN ID of the steer motor.
     */
    int SteerMotorId = 0;
    /**
     * \brief CAN ID of the drive motor.
     */
    int DriveMotorId = 0;
    /**
     * \brief Offset of the azimuth encoder.
     */
    units::angle::turn_t EncoderOffset = 0_tr;
    /**
     * \brief The location of this module's wheels relative to the physical center
     * of the robot in meters along the X axis of the robot.
     */
    units::length::meter_t LocationX = 0_m;
    /**
     * \brief The location of this module's wheels relative to the physical center
     * of the robot in meters along the Y axis of the robot.
     */
    units::length::meter_t LocationY = 0_m;
    /**
     * \brief True if the drive motor is inverted.
     */
    bool DriveMotorInverted = false;
    /**
     * \brief True if the steer motor is inverted from the azimuth. The azimuth
     * should rotate counter-clockwise (as seen from the top of the robot) for a
     * positive motor output.
     */
    bool SteerMotorInverted = false;
    /**
     * \brief True if the azimuth encoder is inverted from the azimuth. The encoder
     * should report a positive velocity when the azimuth rotates counter-clockwise
     * (as seen from the top of the robot).
     */
    bool EncoderInverted = false;
    /**
     * \brief Gear ratio between the drive motor and the wheel.
     */
    units::dimensionless::scalar_t DriveMotorGearRatio = 0;
    /**
     * \brief Gear ratio between the steer motor and the azimuth encoder. For
     * example, the SDS Mk4 has a steering ratio of 12.8.
     */
    units::dimensionless::scalar_t SteerMotorGearRatio = 0;
    /**
     * \brief Coupled gear ratio between the azimuth encoder and the drive motor.
     *
     * For a typical swerve module, the azimuth turn motor also drives the wheel a
     * nontrivial amount, which affects the accuracy of odometry and control. This
     * ratio represents the number of rotations of the drive motor caused by a
     * rotation of the azimuth.
     */
    units::dimensionless::scalar_t CouplingGearRatio = 0;
    /**
     * \brief Radius of the driving wheel in meters.
     */
    units::length::meter_t WheelRadius = 0_m;
    /**
     * \brief The maximum amount of stator current the drive motors can apply
     * without slippage.
     */
    units::current::ampere_t SlipCurrent = 120_A;

    /**
     * \brief The steer motor closed-loop gains.
     *
     * Stupid library requires that you do all of this stupid stuff at in-place memory so here is temp
     *
     * When using closed-loop control, the steer motor uses the control output type
     * specified by #DriveMotorClosedLoopOutput. These gains operate on module rotations
     * (after gear ratio).
     */
    struct Slot0ConfigsRev {
        /**
         * Output rotation is affected by Position Conversion Factor
         *
         * Regular position mode
         *
         * kP: duty cycle per output rotation
         * kI: duty cycle per output rotation * milliseconds
         * kD: duty cycle * milliseconds per output rotation
         *
         * MAXMotion position mode
         *
         * kP: volts per output rotation
         * kI: volts per output rotation * milliseconds
         * kD: volts * milliseconds per output rotation
         */
        double kP, kI, kD;
        /**
         * dFilter: PIDF derivative filter constant
         * iZone: The PIDF loop integrator will only accumulate while the setpoint is within IZone of the target
         * iMaxAccum: Max amount that the PIDF loop integrator will count in area of error towards the target
         */
        double dFilter, iZone, iMaxAccum;
        /**
         * minOut: This is the min output of the controller [-1,1]
         * maxOut: This is the max output of the controller [-1,1]
         */
        double minOut, maxOut;
        /**
         * Sensor from where the input is provided
         */
        rev::spark::FeedbackSensor sensor;
        /**
         * Output velocity is affected by Velocity Conversion Factor
         * Output velocity: rotations per minute
         *
         * cruiseVelocity: output velocity
         * maxAcceleration: output velocity per second
         *
         * Output rotations is affected by Position Conversion Factor
         *
         * allowedError: output rotations
         */
        double cruiseVelocity, maxAcceleration, allowedError;
        /**
         * Output velocity is affected by Velocity Conversion Factor
         * Output velocity: rotations per minute
         *
         * kS: volts
         * kV: volts per output velocity
         * kA: volts per output velocity per second
         */
        double kS, kV, kA;
    } SteerMotorGains;

    /**
     * \brief The drive motor closed-loop gains.
     *
     * When using closed-loop control, the drive motor uses the control output type
     * specified by #DriveMotorClosedLoopOutput. These gains operate on motor rotor rotations
     * (before the gear ratio).
     */
    ctre::phoenix6::configs::Slot0Configs DriveMotorGains = ctre::phoenix6::configs::Slot0Configs{};

    /**
     * \brief When using open-loop drive control, this specifies the speed at which
     * the robot travels when driven with 12 volts. This is used to approximate the
     * output for a desired velocity. If using closed loop control, this value is
     * ignored.
     */
    units::velocity::meters_per_second_t SpeedAt12Volts = 0_mps;
    /**
     * \brief Modifies the SteerMotorId parameter and returns itself.
     *
     * CAN ID of the steer motor.
     *
     * \param newSteerMotorId Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithSteerMotorId(int newSteerMotorId) {
        this->SteerMotorId = newSteerMotorId;
        return *this;
    }

    /**
     * \brief Modifies the DriveMotorId parameter and returns itself.
     *
     * CAN ID of the drive motor.
     *
     * \param newDriveMotorId Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithDriveMotorId(int newDriveMotorId) {
        this->DriveMotorId = newDriveMotorId;
        return *this;
    }

    /**
     * \brief Modifies the EncoderOffset parameter and returns itself.
     *
     * Offset of the azimuth encoder.
     *
     * \param newEncoderOffset Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithEncoderOffset(units::angle::turn_t newEncoderOffset) {
        this->EncoderOffset = newEncoderOffset;
        return *this;
    }

    /**
     * \brief Modifies the LocationX parameter and returns itself.
     *
     * The location of this module's wheels relative to the physical center of the
     * robot in meters along the X axis of the robot.
     *
     * \param newLocationX Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithLocationX(units::length::meter_t newLocationX) {
        this->LocationX = newLocationX;
        return *this;
    }

    /**
     * \brief Modifies the LocationY parameter and returns itself.
     *
     * The location of this module's wheels relative to the physical center of the
     * robot in meters along the Y axis of the robot.
     *
     * \param newLocationY Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithLocationY(units::length::meter_t newLocationY) {
        this->LocationY = newLocationY;
        return *this;
    }

    /**
     * \brief Modifies the DriveMotorInverted parameter and returns itself.
     *
     * True if the drive motor is inverted.
     *
     * \param newDriveMotorInverted Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithDriveMotorInverted(bool newDriveMotorInverted) {
        this->DriveMotorInverted = newDriveMotorInverted;
        return *this;
    }

    /**
     * \brief Modifies the SteerMotorInverted parameter and returns itself.
     *
     * True if the steer motor is inverted from the azimuth. The azimuth should
     * rotate counter-clockwise (as seen from the top of the robot) for a positive
     * motor output.
     *
     * \param newSteerMotorInverted Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithSteerMotorInverted(bool newSteerMotorInverted) {
        this->SteerMotorInverted = newSteerMotorInverted;
        return *this;
    }

    /**
     * \brief Modifies the EncoderInverted parameter and returns itself.
     *
     * True if the azimuth encoder is inverted from the azimuth. The encoder should
     * report a positive velocity when the azimuth rotates counter-clockwise (as
     * seen from the top of the robot).
     *
     * \param newEncoderInverted Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithEncoderInverted(bool newEncoderInverted) {
        this->EncoderInverted = newEncoderInverted;
        return *this;
    }

    /**
     * \brief Modifies the DriveMotorGearRatio parameter and returns itself.
     *
     * Gear ratio between the drive motor and the wheel.
     *
     * \param newDriveMotorGearRatio Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithDriveMotorGearRatio(units::dimensionless::scalar_t newDriveMotorGearRatio) {
        this->DriveMotorGearRatio = newDriveMotorGearRatio;
        return *this;
    }

    /**
     * \brief Modifies the SteerMotorGearRatio parameter and returns itself.
     *
     * Gear ratio between the steer motor and the azimuth encoder. For example, the
     * SDS Mk4 has a steering ratio of 12.8.
     *
     * \param newSteerMotorGearRatio Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithSteerMotorGearRatio(units::dimensionless::scalar_t newSteerMotorGearRatio) {
        this->SteerMotorGearRatio = newSteerMotorGearRatio;
        return *this;
    }

    /**
     * \brief Modifies the CouplingGearRatio parameter and returns itself.
     *
     * Coupled gear ratio between the azimuth encoder and the drive motor.
     *
     * For a typical swerve module, the azimuth turn motor also drives the wheel a
     * nontrivial amount, which affects the accuracy of odometry and control. This
     * ratio represents the number of rotations of the drive motor caused by a
     * rotation of the azimuth.
     *
     * \param newCouplingGearRatio Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithCouplingGearRatio(units::dimensionless::scalar_t newCouplingGearRatio) {
        this->CouplingGearRatio = newCouplingGearRatio;
        return *this;
    }

    /**
     * \brief Modifies the WheelRadius parameter and returns itself.
     *
     * Radius of the driving wheel in meters.
     *
     * \param newWheelRadius Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithWheelRadius(units::length::meter_t newWheelRadius) {
        this->WheelRadius = newWheelRadius;
        return *this;
    }

    /**
     * \brief Modifies the SteerMotorGains parameter and returns itself.
     *
     * The steer motor closed-loop gains.
     *
     * Stupid library requires that you do all of this stupid stuff at in-place memory so here is temp
     *
     * When using closed-loop control, the steer motor uses the control output type
     * specified by #DriveMotorClosedLoopOutput. These gains operate on module rotations
     * (after gear ratio).
     *
     * \param newSteerMotorGains Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithSteerMotorGains(const Slot0ConfigsRev& newSteerMotorGains) {
        this->SteerMotorGains = newSteerMotorGains;
        return *this;
    }

    /**
     * \brief Modifies the DriveMotorGains parameter and returns itself.
     *
     * The drive motor closed-loop gains.
     *
     * When using closed-loop control, the drive motor uses the control output type
     * specified by #DriveMotorClosedLoopOutput and any closed-loop
     * SwerveModule#DriveRequestType. These gains operate on motor rotor rotations
     * (before the gear ratio).
     *
     * \param newDriveMotorGains Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithDriveMotorGains(const ctre::phoenix6::configs::Slot0Configs& newDriveMotorGains) {
        this->DriveMotorGains = newDriveMotorGains;
        return *this;
    }

    /**
     * \brief Modifies the SlipCurrent parameter and returns itself.
     *
     * The maximum amount of stator current the drive motors can apply without
     * slippage.
     *
     * \param newSlipCurrent Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithSlipCurrent(units::current::ampere_t newSlipCurrent) {
        this->SlipCurrent = newSlipCurrent;
        return *this;
    }

    /**
     * \brief Modifies the SpeedAt12Volts parameter and returns itself.
     *
     * When using open-loop drive control, this specifies the speed at which the
     * robot travels when driven with 12 volts. This is used to approximate the
     * output for a desired velocity. If using closed loop control, this value is
     * ignored.
     *
     * \param newSpeedAt12Volts Parameter to modify
     * \returns this object
     */
    constexpr SwerveModuleConstants& WithSpeedAt12Volts(units::velocity::meters_per_second_t newSpeedAt12Volts) {
        this->SpeedAt12Volts = newSpeedAt12Volts;
        return *this;
    }
};

#endif