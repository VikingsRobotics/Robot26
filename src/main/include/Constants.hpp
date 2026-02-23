#pragma once
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <pathplanner/lib/config/PIDConstants.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>

#include <wpi/json.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/ClosedLoopConfig.h>
#include <rev/AbsoluteEncoder.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <units/frequency.h>
#include <units/time.h>

#include <units/length.h>
#include <units/voltage.h>

#include <numbers>

#include "constants/SwerveModuleConfig.hpp"

namespace DeviceIdentifier {
// CTRE: CANBus Name for contructors of CRTE software classes
constexpr ctre::phoenix6::CANBus kCANBus{""};
// REV: PDH
constexpr int kPDHId = 1;

// CTRE: CANBus Pigeon2 ID
constexpr int kGyroId = 3;
// CTRE: Falcon 500 Front Left Motor ID
constexpr int kFLDriveMotorId = 6;
// REV: Neo 550 Front Left Angle Motor ID
constexpr int kFLAngleMotorId = 7;
// CTRE: Falcon 500 Front Right Motor ID
constexpr int kFRDriveMotorId = 8;
// REV: Neo 550 Front Right Angle Motor ID
constexpr int kFRAngleMotorId = 9;
// CTRE: Falcon 500 Back Left Motor ID
constexpr int kBLDriveMotorId = 4;
// REV: Neo 550 Back Left Angle Motor ID
constexpr int kBLAngleMotorId = 5;
// CTRE: Falcon 500 Back Right Motor ID
constexpr int kBRDriveMotorId = 10;
// REV: Neo 550 Back Right Angle Motor ID
constexpr int kBRAngleMotorId = 11;
}  // namespace DeviceIdentifier

namespace Swerve {

namespace Mechanism {
using turns_per_min_t = units::unit_t<units::compound_unit<units::angle::turn, units::inverse<units::time::minute>>>;
using meters_per_turn_t = units::unit_t<units::compound_unit<units::length::meters, units::inverse<units::angle::turns>>>;
// Motor pinion gear teeth
constexpr units::dimensionless::scalar_t kDrivingPinion{13};
// Motor spur gear teeth
constexpr units::dimensionless::scalar_t kDrivingSpur{22};
// Wheel transmission gear teeth
constexpr units::dimensionless::scalar_t kTransmission{15};
// Wheel gear teeth
constexpr units::dimensionless::scalar_t kWheel{45};
// Full drive motor to wheel ratio
constexpr units::dimensionless::scalar_t kDriveGearRatio{(kDrivingSpur * kWheel) / (kDrivingPinion * kTransmission)};
// Expected amount of drive wheel movement after rotating azimuth
constexpr units::dimensionless::scalar_t kCouplingRatio{kDrivingPinion / kDrivingSpur};
// Wheel diameter
constexpr units::inch_t kWheelDiameter{3};
// Wheel circumference
constexpr units::meter_t kWheelCircumference{kWheelDiameter * std::numbers::pi};
// Distance to rotation
constexpr meters_per_turn_t kDriveMotorToDistance{kWheelCircumference / (1_tr * kDriveGearRatio)};

// Gear ratio reduction between azimuth moter and pinion
constexpr units::dimensionless::scalar_t kAzimuthReduction{(76.0 * 84.0) / (21.0 * 29.0)};
// Azimuth pinion teeth
constexpr units::dimensionless::scalar_t kAzimuthPinion{14};
// Azimuth steering gear teeth
constexpr units::dimensionless::scalar_t kSteer{62};
// Full azimuth motor to module rotation
constexpr units::dimensionless::scalar_t kAzimuthGearRatio{(kAzimuthReduction * kSteer) / kAzimuthPinion};

// Max speed of drive motors
constexpr turns_per_min_t kDriveRPM{6380};
// Max speed of azimuth motors
constexpr turns_per_min_t kAzimuthRPM{11000};
// Max speed a module can theoretically move
constexpr units::meters_per_second_t kMaxMovement{kDriveRPM * kWheelCircumference / (1_tr * kDriveGearRatio)};
// Max speed a module can theoretically rotate
constexpr units::turns_per_second_t kMaxRotation{kAzimuthRPM / kAzimuthGearRatio};
// How long the base of the robot is
constexpr units::inch_t kWheelBase = 27.5_in;
// How wide the base of the robot is
constexpr units::inch_t kTrackWidth = 27.5_in;
}  // namespace Mechanism

namespace DeviceProperties {
inline SwerveModuleConfigs kFrontLeft =
    SwerveModuleConfigs{DeviceIdentifier::kFLDriveMotorId, DeviceIdentifier::kFLAngleMotorId}
        .WithAzimuthGearRatio(Mechanism::kAzimuthGearRatio)
        .WithDriveGearRatio(Mechanism::kDriveGearRatio)
        .WithCouplingRatio(Mechanism::kCouplingRatio)
        .WithDriveWheelCircumference(Mechanism::kWheelCircumference)
        .WithMaxDriveMotorSpeed(Mechanism::kMaxMovement)
        .WithMaxDriveMotorAcceleration(5_mps_sq)
        .WithMaxDriveMotorJerk(SwerveModuleConfigs::meters_per_sec_cu_t{5})
        .WithMaxAzimuthMotorSpeed(Mechanism::kMaxRotation)
        .WithMaxAzimuthMotorAcceleration(3.9_tr_per_s_sq)
        .WithInvertedEncoder(true)
        .WithMusicDuringDisable(true)
        .WithResetPositionOnConfig(true)
        .WithAzimuthMotorPID(Gains::DutyCyclePIDRotationMs{units::dimensionless::scalar_t{1} / 1_tr, units::dimensionless::scalar_t{0} / (1_tr * 1_ms),
                                                           units::dimensionless::scalar_t{0} / (1_tr / 1_ms)})
        .WithAzimuthMotorFF(Gains::VoltageFFRevolution{0_V, 0_V / 1_rpm, 0_V / 1_rev_per_m_per_s})
        .WithDriveMagicVA((12_V * 1_s) / 108_m, (0_V * 1_s * 1_s) / 1_m)
        .WithDriveMotorDutyGain(Gains::DutyCycleGainDistance{
            Gains::DutyCyclePIDDistance{units::dimensionless::scalar_t{0.0833} / 1_m, units::dimensionless::scalar_t{0} / (1_m * 1_s),
                                        units::dimensionless::scalar_t{0} / (1_m / 1_s)},
            Gains::DutyCycleFFDistance{units::dimensionless::scalar_t{0}, units::dimensionless::scalar_t{0} / 1_mps,
                                       units::dimensionless::scalar_t{0} / 1_mps_sq}})
        .WithDriveMotorVoltageGain(Gains::VoltageGainDistance{Gains::VoltagePIDDistance{1_V / 1_m, 0_V / (1_m * 1_s), 0_V / (1_m / 1_s)},
                                                              Gains::VoltageFFDistance{0_V, 0_V / 1_mps, 0_V / 1_mps_sq}})
        .WithDriveMotorTorqueGain(Gains::TorqueGainDistance{Gains::TorquePIDDistance{21.4166_A / 1_m, 0_A / (1_m * 1_s), 0_A / (1_m / 1_s)},
                                                            Gains::TorqueFFDistance{0_A, 0_A / 1_mps, 0_A / 1_mps_sq}})
        .WithModuleOffset(frc::Translation2d{+Mechanism::kWheelBase / 2, +Mechanism::kTrackWidth / 2})
        .WithRotationalOffset(-0.25_tr);

inline SwerveModuleConfigs kFrontRight = SwerveModuleConfigs{DeviceIdentifier::kFRDriveMotorId, DeviceIdentifier::kFRAngleMotorId}
                                             .Apply(kFrontLeft)
                                             .WithModuleOffset(frc::Translation2d{+Mechanism::kWheelBase / 2, -Mechanism::kTrackWidth / 2})
                                             .WithRotationalOffset(0_tr);

inline SwerveModuleConfigs kBackLeft = SwerveModuleConfigs{DeviceIdentifier::kBLDriveMotorId, DeviceIdentifier::kBLAngleMotorId}
                                           .Apply(kFrontLeft)
                                           .WithModuleOffset(frc::Translation2d{-Mechanism::kWheelBase / 2, +Mechanism::kTrackWidth / 2})
                                           .WithRotationalOffset(0.5_tr);

inline SwerveModuleConfigs kBackRight = SwerveModuleConfigs{DeviceIdentifier::kBRDriveMotorId, DeviceIdentifier::kBRAngleMotorId}
                                            .Apply(kFrontLeft)
                                            .WithModuleOffset(frc::Translation2d{-Mechanism::kWheelBase / 2, -Mechanism::kTrackWidth / 2})
                                            .WithRotationalOffset(0.25_tr);

}  // namespace DeviceProperties

namespace System {
extern frc::SwerveDriveKinematics<4> kDriveKinematics;
}

namespace TeleopOperator {
// USB ID for joystick for driver
constexpr int kDriverControllerPort = 0;
// Slew rate limiter for translation
constexpr units::hertz_t kTransLimiter = 1 / 1_s;
// Slew rate limiter for azimuth
constexpr units::hertz_t kAngleLimiter = 1 / 1_s;
// Slew rate limiter for translation magitude
constexpr units::hertz_t kMagLimiter = 1 / 1_s;
// Slew rate limiter for translation direction
constexpr units::hertz_t kDirLimiter = 1 / 1_s;


// Minimum percent of joystick distance before robot response (move)
constexpr double kDriveDeadband = 0.15;
// Minimum percent of joystick twist distance before robot response (angle)
constexpr double kDriveAngleDeadband = 0.15;
// Maximum speed that the robot will move (limited by physical design)
constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;

// Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedMax = 3.0_rad_per_s;
// Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_squared_t kDriveAngleAccelMax = 5.0_rad_per_s_sq;

inline Gains::PIDConstants<units::radians_per_second_t,units::radian_t,units::second_t> kLookPID {
    0.8_rad_per_s / 1_rad,
    0.0_rad_per_s / (1.0_rad * 1_s),
    0.0_rad_per_s / (1.0_rad / 1_s)
};
}  // namespace TeleopOperator

namespace Auto {
// Translation PID Values (PathplannerLib)
constexpr pathplanner::PIDConstants kTranslationPID{1.0, 0.0, 0.0};
// Rotation PID Values (PathplannerLib)
constexpr pathplanner::PIDConstants kRotationalPID{1.0, 0.0, 0.0};
// Max speed that PathplannerLib can use on the robot (limited by physical
// design)
constexpr units::meters_per_second_t kMaxSpeed = 5.0_mps;
// Max acceleration that Pathplanner can use on the robot (limited by physical
// design)
constexpr units::meters_per_second_squared_t kMaxAcceleration = 3.0_mps_sq;
// Max angular speed that PathplannerLib can use on the robot (limited by
// physical design)
constexpr units::radians_per_second_t kMaxAngularSpeed = 3.0_rad_per_s;
// Max angular acceleration that PathplannerLib can use on the robot (limited by
// physical design)
constexpr units::radians_per_second_squared_t kMaxAngularAcceleration = 5.0_rad_per_s_sq;
}  // namespace Auto

}  // namespace Swerve

#endif
