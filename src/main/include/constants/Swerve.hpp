#pragma once
#ifndef CONSTANTS_SWERVE_H
#define CONSTANTS_SWERVE_H

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

}  // namespace Mechanism

namespace Characterization {
using volt_sec_per_meter_t = units::unit_t<units::compound_unit<units::voltage::volt, units::time::second, units::inverse<units::length::meter>>>;
using volt_square_sec_per_meter_t =
    units::unit_t<units::compound_unit<units::voltage::volt, units::squared<units::time::second>, units::inverse<units::length::meter>>>;
using volt_sec_per_turn_t = units::unit_t<units::compound_unit<units::voltage::volt, units::time::second, units::inverse<units::angle::turn>>>;
using volt_square_sec_per_turn_t =
    units::unit_t<units::compound_unit<units::voltage::volt, units::squared<units::time::second>, units::inverse<units::angle::turn>>>;

namespace Drive {
// Static voltage for driving
constexpr units::volt_t kS{0.15_V};
// Velocity voltage for driving
constexpr volt_sec_per_meter_t kV{(12_V * 1_s) / (1_m * ((Mechanism::kDriveRPM * 1_s) / 1_tr))};
// Acceleration voltage for driving
constexpr volt_square_sec_per_meter_t kA{(0_V * 0_s * 0_s) / 1_m};
// Proportional value for driving
constexpr units::dimensionless::scalar_t kP{0.1};
// Integral value for driving
constexpr units::dimensionless::scalar_t kI{0};
// Derivative value for driving
constexpr units::dimensionless::scalar_t kD{0};
}  // namespace Drive

namespace Azimuth {
// Static voltage for azimuth
constexpr units::volt_t kS{0_V};
// Velocity voltage for azimuth
constexpr volt_sec_per_turn_t kV{(0_V * 0_s) / 1_tr};
// Acceleration voltage for azimuth
constexpr volt_square_sec_per_turn_t kA{(0_V * 0_s * 0_s) / 1_tr};
// Proportional value for azimuth
constexpr units::dimensionless::scalar_t kP{1};
// Integral value for azimuth
constexpr units::dimensionless::scalar_t kI{0};
// Derivative value for azimuth
constexpr units::dimensionless::scalar_t kD{0};
}  // namespace Azimuth

}  // namespace Characterization


namespace DeviceProperties {
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetSparkMaxConfig();
// Default motor type used for TalonFX motors
extern ctre::phoenix6::configs::TalonFXConfiguration GetTalonFXConfig();
// Default motor type enum for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
// Invert absolute encoder to match direction of motor movement
constexpr bool kInvertEncoder = true;
}  // namespace DeviceProperties

namespace System {
extern frc::SwerveDriveKinematics<4> kDriveKinematics;
}

namespace TeleopOperator {
// USB ID for joystick for driver
constexpr int kDriverControllerPort = 0;
// Slew rate limiter for translation
constexpr units::hertz_t kLimiter = 1 / 1_s;
// Minimum percent of joystick distance before robot response (move)
constexpr double kDriveDeadband = 0.15;
// Minimum percent of joystick twist distance before robot response (angle)
constexpr double kDriveAngleDeadband = 0.15;
// Maximum speed that the robot will move (limited by physical design)
constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;
// Speed when speed throttle is less that precision throttle threshold
constexpr units::meters_per_second_t kDrivePrecision = 0.6_mps;
// What the throttle value will be on startup
constexpr double kDefaultThrottleXbox = 0.7;
// Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedMax = 3.0_rad_per_s;
// Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedPrecision = 0.5_rad_per_s;

constexpr units::second_t kDebounce = 0.1_s;
constexpr units::second_t kVisionDebounce = 0.35_s;
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
