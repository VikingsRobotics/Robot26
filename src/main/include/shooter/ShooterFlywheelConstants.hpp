#ifndef FLYWHEEL_CONSTANT_H
#define FLYWHEEL_CONSTANT_H
#pragma once

#include <units/mass.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/current.h>
#include <units/angular_velocity.h>

#include <frc/geometry/Pose3d.h>

#include <rev/config/ClosedLoopConfig.h>

struct ShooterFlywheelConstants {
    constexpr ShooterFlywheelConstants() = default;
    constexpr ShooterFlywheelConstants(const ShooterFlywheelConstants&) = default;
    /**
     * \brief CAN ID of the flywheel motor.
     */
    int MotorId = 0;
    /**
     * \brief The location of this flywheel relative to the physical center
     * of the robot.
     */
    frc::Transform3d Location{};
    /**
     * \brief True if the flywheel motor is inverted.
     */
    bool MotorInverted = false;
    /**
     * \brief Gear ratio between the flywheel motor and the wheel.
     */
    units::dimensionless::scalar_t MotorGearRatio = 0;
    /**
     * \brief Radius of the flywheel in meters.
     */
    units::length::meter_t WheelRadius = 0_m;
    /**
     * \brief Mass of the flywheel in kilograms.
     */
    units::mass::kilogram_t WheelMass = 0_kg;
    /**
     * \brief The maximum amount of stator current the flywheel can apply
     * without slippage.
     */
    units::current::ampere_t SlipCurrent = 120_A;

    /**
     * \brief The steer motor closed-loop gains.
     *
     * Stupid library requires that you do all of this stupid stuff at in-place memory so here is temp
     *
     * When using closed-loop control, the flywheel motor uses these gains. These gains operate on flywheel rotations
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
         * posWrapEnabled: Enables the controller to assume that posMinInput and posMaxInput are the same point
         */
        bool posWrapEnabled;
        /**
         * posMinInput: Min value that the controller can output during position wrapped input
         * posMinInput: Max value that the controller can output during position wrapped input
         */
        double posMinInput, posMaxInput;
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
    } MotorGains;

    /**
     * \brief When using open-loop flywheel control, this specifies the speed at which
     * the flywheel rotates when commanded with 12 volts. This is used to approximate the
     * output for a desired velocity. If using closed loop control, this value is
     * ignored.
     */
    units::angular_velocity::turns_per_second_t SpeedAt12Volts = 0_tps;
    /**
     * \brief Modifies the MotorId parameter and returns itself.
     *
     * CAN ID of the flywheel motor.
     *
     * \param newMotorId Parameter to modify
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithMotorId(int newMotorId) {
        this->MotorId = newMotorId;
        return *this;
    }

    /**
     * \brief Modifies the Location parameter and returns itself.
     *
     * The location of this flywheel relative to the physical center
     * of the robot.
     *
     * \param newLocation New flywheel pose
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithLocation(const frc::Transform3d& newLocation) {
        this->Location = newLocation;
        return *this;
    }

    /**
     * \brief Modifies the MotorInverted parameter and returns itself.
     *
     * True if the flywheel drive motor should be inverted.
     *
     * \param inverted Whether the motor is inverted
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithMotorInverted(bool inverted) {
        this->MotorInverted = inverted;
        return *this;
    }

    /**
     * \brief Modifies the MotorGearRatio parameter and returns itself.
     *
     * Gear ratio between the motor and the flywheel.
     *
     * \param ratio Gear reduction ratio
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithMotorGearRatio(units::dimensionless::scalar_t ratio) {
        this->MotorGearRatio = ratio;
        return *this;
    }

    /**
     * \brief Modifies the WheelRadius parameter and returns itself.
     *
     * Radius of the flywheel.
     *
     * \param radius Flywheel radius
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithWheelRadius(units::length::meter_t radius) {
        this->WheelRadius = radius;
        return *this;
    }

    /**
     * \brief Modifies the WheelMass parameter and returns itself.
     *
     * Mass of the flywheel.
     *
     * \param mass Flywheel mass
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithWheelMass(units::mass::kilogram_t mass) {
        this->WheelMass = mass;
        return *this;
    }

    /**
     * \brief Modifies the SlipCurrent parameter and returns itself.
     *
     * Maximum stator current that can be applied before the flywheel
     * begins to slip on the game piece.
     *
     * \param current Maximum slip current
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithSlipCurrent(units::current::ampere_t current) {
        this->SlipCurrent = current;
        return *this;
    }

    /**
     * \brief Modifies the MotorGains parameter and returns itself.
     *
     * Closed-loop control gains for the motor controller.
     *
     * \param gains Closed-loop gain configuration
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithMotorGains(const Slot0ConfigsRev& gains) {
        this->MotorGains = gains;
        return *this;
    }

    /**
     * \brief Modifies the SpeedAt12Volts parameter and returns itself.
     *
     * The theoretical free speed of the flywheel when driven at 12 volts.
     * Used for open-loop velocity approximation.
     *
     * \param speed Speed at 12 volts
     * \returns this object
     */
    constexpr ShooterFlywheelConstants& WithSpeedAt12Volts(units::angular_velocity::turns_per_second_t speed) {
        this->SpeedAt12Volts = speed;
        return *this;
    }
};

#endif