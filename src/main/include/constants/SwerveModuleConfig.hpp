#pragma once
#ifndef SUBSYSTEM_MODULE_CONFIG_H
#define SUBSYSTEM_MODULE_CONFIG_H

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>
#include <units/current.h>

#include <frc/geometry/Translation2d.h>

#include "constants/GainConstants.hpp"

/**
 * @class SwerveModuleConfigs
 *
 * @brief Configuration container for a single swerve module.
 *
 * This class encapsulates all mechanical ratios, feedforward constants,
 * PID gains, motion constraints, and behavioral flags required to
 * configure a swerve drive module.
 *
 * This class once initialized with ids for drive and azimuth motors,
 * will not allow changes to the ids for drive and azimuth motors. All
 * other configuration modification will take place after apply the
 * configuration to the swerve module.
 *
 * It is designed to be:
 *  - Strongly typed using WPILib units
 *  - Immutable-by-convention via fluent `WithX()` setters
 *  - `constexpr`-friendly for compile-time configuration
 *
 */
class SwerveModuleConfigs {
public:
    /** Distance position in turns */
    using meter_t = units::meter_t;
    /** Angular position in turns */
    using turn_t = units::turn_t;
    /** Dimensionless scalar (used for PID gains) */
    using scalar_t = units::dimensionless::scalar_t;
    /** Gear ratio expressed as motor turns per mechanism turn */
    using turns_per_turn_t = units::dimensionless::scalar_t;
    /** Linear distance traveled per motor rotation */
    using meters_per_turn_t = units::unit_t<units::compound_unit<units::length::meters, units::inverse<units::angle::turns>>>;
    /** Linear velocity */
    using meters_per_sec_t = units::meters_per_second_t;
    /** Linear acceleration */
    using meters_per_sec_sqr_t = units::meters_per_second_squared_t;
    /** Linear jerk */
    using meters_per_sec_cu_t = units::unit_t<units::compound_unit<units::meters, units::inverse<units::cubed<units::seconds>>>>;
    /** Angular velocity */
    using turns_per_sec_t = units::turns_per_second_t;
    /** Angular acceleration */
    using turns_per_sec_sqr_t = units::turns_per_second_squared_t;
    /** Drive feedforward velocity gain */
    using volts_per_meter_per_sec_t = units::unit_t<units::compound_unit<units::voltage::volt, units::inverse<units::meters_per_second>>>;
    /** Drive feedforward acceleration gain */
    using volts_per_meter_per_sec_sqr_t = units::unit_t<units::compound_unit<units::voltage::volt, units::inverse<units::meters_per_second_squared>>>;

public:
    /** Drive motor CAN bus id */
    const int kDriveId;

    /** Azimuth motor CAN bus id */
    const int kAzimuthId;


    /** Gear ratios assume that values greater than zero are gear reduction */

    /** Drive motor gear ratio (motor turns per wheel turn) */
    turns_per_turn_t kDriveGearRatio{1};
    /** Mechanical coupling ratio between drive and azimuth, if present */
    turns_per_turn_t kCouplingRatio{1};
    /** Azimuth motor gear ratio (motor turns per module rotation) */
    turns_per_turn_t kAzimuthGearRatio{1};
    /** Wheel circumference */
    meter_t kWheelCircumference{1};


    /** CALCULATED: Drive distance per motor rotation */
    meters_per_turn_t kDriveRotationToOutputDistance{1};
    /** CALCULATED: Drive distance per geared rotation */
    meters_per_turn_t kGearedRotationToOutputDistance{1};
    /** Module rotation per azimuth motor turn */
    turns_per_turn_t kAzimuthRotationToOutputRotation{1};

    /** Drive duty cycle gains */
    Gains::DutyCycleGainDistance kDriveDutyGain{Gains::DutyCyclePIDDistance{scalar_t{0.1} / 1_m, scalar_t{0} / (1_m * 1_s), scalar_t{0} / (1_m / 1_s)},
                                                Gains::DutyCycleFFDistance{scalar_t{0}, scalar_t{0} / 1_mps, scalar_t{0} / 1_mps_sq}};

    /** Drive voltage gains */
    Gains::VoltageGainDistance kDriveVoltageGain{Gains::VoltagePIDDistance{1_V / 1_m, 0_V / (1_m * 1_s), 0_V / (1_m / 1_s)},
                                                 Gains::VoltageFFDistance{0_V, 0_V / 1_mps, 0_V / 1_mps_sq}};

    /** Drive torque gains */
    Gains::TorqueGainDistance kDriveTorqueGain{Gains::TorquePIDDistance{0.083_A / 1_m, 0_A / (1_m * 1_s), 0_A / (1_m / 1_s)},
                                               Gains::TorqueFFDistance{0_A, 0_A / 1_mps, 0_A / 1_mps_sq}};

    /** Drive velocity feedforward (kV) for motion magic */
    volts_per_meter_per_sec_t kVMagicDrive{0};
    /** Drive acceleration feedforward (kA) for motion magic */
    volts_per_meter_per_sec_sqr_t kAMagicDrive{0};

    /** Azimuth pid */
    Gains::DutyCyclePIDRotationMs kAzimuthPID{scalar_t{1} / 1_tr, scalar_t{0} / (1_tr * 1_ms), scalar_t{0} / (1_tr / 1_ms)};

    /** Azimuth ff */
    Gains::VoltageFFRevolution kAzimuthFF{0_V, 0_V / 1_rpm, 0_V / 1_rev_per_m_per_s};

    /** Maximum allowed drive speed */
    meters_per_sec_t kMaxDriveSpeed{5};
    /** Maximum allowed drive acceleration */
    meters_per_sec_sqr_t kMaxDriveAccel{5};
    /** Maximun allowed drive jerk */
    meters_per_sec_cu_t kMaxDriveJerk{5};

    /** Maximum allowed azimuth angular speed */
    turns_per_sec_t kMaxAzimuthSpeed{5};
    /** Maximum allowed azimuth angular acceleration */
    turns_per_sec_sqr_t kMaxAzimuthAccel{5};

    /** Absolute encoder offset for module zeroing */
    turn_t kRotationOffset{0};

    /** Translation of this module from the center of the robot (angle measurement) */
    frc::Translation2d kModuleTranslation{0_m, 0_m};

    /** Invert encoder direction */
    bool kInvertEncoder{false};
    /** Reset encoder position when configuration is applied */
    bool kResetEncoderOnConfig{true};
    /** Allow motor controller music during disabled mode */
    bool kAllowMusicDuringDisable{true};

public:
    /**
     * @brief Construct a swerve module configuration with required parameters.
     *
     * @param driveId Drive motor CAN bus id
     * @param azimuthId Azimuth motor CAN bus id
     */
    SwerveModuleConfigs(int driveId, int azimuthId) : kDriveId(driveId), kAzimuthId(azimuthId) {}

    /** @brief Apply the configuration of another without changing ids */
    SwerveModuleConfigs& Apply(const SwerveModuleConfigs& configs) {
        // Copy all non-const members
        kDriveGearRatio = configs.kDriveGearRatio;
        kCouplingRatio = configs.kCouplingRatio;
        kAzimuthGearRatio = configs.kAzimuthGearRatio;
        kWheelCircumference = configs.kWheelCircumference;

        kDriveRotationToOutputDistance = configs.kDriveRotationToOutputDistance;
        kGearedRotationToOutputDistance = configs.kGearedRotationToOutputDistance;
        kAzimuthRotationToOutputRotation = configs.kAzimuthRotationToOutputRotation;

        kDriveDutyGain = configs.kDriveDutyGain;
        kDriveVoltageGain = configs.kDriveVoltageGain;
        kDriveTorqueGain = configs.kDriveTorqueGain;

        kVMagicDrive = configs.kVMagicDrive;
        kAMagicDrive = configs.kAMagicDrive;

        kAzimuthPID = configs.kAzimuthPID;
        kAzimuthFF = configs.kAzimuthFF;

        kMaxDriveSpeed = configs.kMaxDriveSpeed;
        kMaxDriveAccel = configs.kMaxDriveAccel;
        kMaxDriveJerk = configs.kMaxDriveJerk;

        kMaxAzimuthSpeed = configs.kMaxAzimuthSpeed;
        kMaxAzimuthAccel = configs.kMaxAzimuthAccel;

        kRotationOffset = configs.kRotationOffset;
        kModuleTranslation = configs.kModuleTranslation;

        kInvertEncoder = configs.kInvertEncoder;
        kResetEncoderOnConfig = configs.kResetEncoderOnConfig;
        kAllowMusicDuringDisable = configs.kAllowMusicDuringDisable;

        return *this;
    }


    /** @brief Set the drive gear ratio. */
    SwerveModuleConfigs& WithDriveGearRatio(turns_per_turn_t driveGearRatio) {
        kDriveGearRatio = driveGearRatio;
        kDriveRotationToOutputDistance = kWheelCircumference / (kDriveGearRatio * 1_tr);
        return *this;
    }

    /** @brief Set the mechanical coupling ratio. */
    SwerveModuleConfigs& WithCouplingRatio(turns_per_turn_t couplingRatio) {
        kCouplingRatio = couplingRatio;
        return *this;
    }

    /** @brief Set the azimuth gear ratio. */
    SwerveModuleConfigs& WithAzimuthGearRatio(turns_per_turn_t azimuthGearRatio) {
        kAzimuthGearRatio = azimuthGearRatio;
        return *this;
    }
    /** @brief Set wheel circumference. */
    SwerveModuleConfigs& WithDriveWheelCircumference(meter_t circumference) {
        kWheelCircumference = circumference;
        kGearedRotationToOutputDistance = kWheelCircumference / 1_tr;
        kDriveRotationToOutputDistance = kWheelCircumference / (kDriveGearRatio * 1_tr);
        return *this;
    }
    /** @brief Set azimuth rotation to output rotation. */
    SwerveModuleConfigs& WithAzimuthRotationToOutputRotation(turns_per_turn_t azimuthRotationToOutputRotation) {
        kAzimuthRotationToOutputRotation = azimuthRotationToOutputRotation;
        return *this;
    }

    /** @brief Set drive duty cycle gains. */
    SwerveModuleConfigs& WithDriveMotorDutyGain(Gains::DutyCycleGainDistance gain) {
        kDriveDutyGain = gain;
        return *this;
    }
    /** @brief Set drive voltage gains. */
    SwerveModuleConfigs& WithDriveMotorVoltageGain(Gains::VoltageGainDistance gain) {
        kDriveVoltageGain = gain;
        return *this;
    }
    /** @brief Set drive torque gains. */
    SwerveModuleConfigs& WithDriveMotorTorqueGain(Gains::TorqueGainDistance gain) {
        kDriveTorqueGain = gain;
        return *this;
    }
    /** @brief Set drive velocity feedforward (kV) for motion magic. */
    SwerveModuleConfigs& WithDriveMagickV(volts_per_meter_per_sec_t kV) {
        kVMagicDrive = kV;
        return *this;
    }
    /** @brief Set drive acceleration feedforward (kA) for motion magic. */
    SwerveModuleConfigs& WithDriveMagickA(volts_per_meter_per_sec_sqr_t kA) {
        kAMagicDrive = kA;
        return *this;
    }
    /** @brief Set motion magic drive feedforward constants. */
    SwerveModuleConfigs& WithDriveMagicVA(volts_per_meter_per_sec_t kV, volts_per_meter_per_sec_sqr_t kA) {
        kVMagicDrive = kV;
        kAMagicDrive = kA;
        return *this;
    }
    /** @brief Set azimuth duty cycle gains. */
    SwerveModuleConfigs& WithAzimuthMotorPID(Gains::DutyCyclePIDRotationMs pid) {
        kAzimuthPID = pid;
        return *this;
    }
    /** @brief Set azimuth voltage gains. */
    SwerveModuleConfigs& WithAzimuthMotorFF(Gains::VoltageFFRevolution ff) {
        kAzimuthFF = ff;
        return *this;
    }
    /** @brief Set maximum drive speed. */
    SwerveModuleConfigs& WithMaxDriveMotorSpeed(meters_per_sec_t speed) {
        kMaxDriveSpeed = speed;
        return *this;
    }
    /** @brief Set maximum drive acceleration. */
    SwerveModuleConfigs& WithMaxDriveMotorAcceleration(meters_per_sec_sqr_t accel) {
        kMaxDriveAccel = accel;
        return *this;
    }
    /** @brief Set maximum drive acceleration. */
    SwerveModuleConfigs& WithMaxDriveMotorJerk(meters_per_sec_cu_t jerk) {
        kMaxDriveJerk = jerk;
        return *this;
    }
    /** @brief Set maximum azimuth speed. */
    SwerveModuleConfigs& WithMaxAzimuthMotorSpeed(turns_per_sec_t speed) {
        kMaxAzimuthSpeed = speed;
        return *this;
    }
    /** @brief Set maximum azimuth acceleration. */
    SwerveModuleConfigs& WithMaxAzimuthMotorAcceleration(turns_per_sec_sqr_t accel) {
        kMaxAzimuthAccel = accel;
        return *this;
    }
    /** @brief Set absolute rotational offset. */
    SwerveModuleConfigs& WithRotationalOffset(turn_t offset) {
        kRotationOffset = offset;
        return *this;
    }
    /** @brief Set module translation. */
    SwerveModuleConfigs& WithModuleOffset(frc::Translation2d offset) {
        kModuleTranslation = offset;
        return *this;
    }
    /** @brief Invert encoder direction. */
    SwerveModuleConfigs& WithInvertedEncoder(bool invert) {
        kInvertEncoder = invert;
        return *this;
    }
    /** @brief Reset encoder on configuration apply. */
    SwerveModuleConfigs& WithResetPositionOnConfig(bool reset) {
        kResetEncoderOnConfig = reset;
        return *this;
    }
    /** @brief Allow motor controller music while disabled. */
    SwerveModuleConfigs& WithMusicDuringDisable(bool enable) {
        kAllowMusicDuringDisable = enable;
        return *this;
    }
};


#endif