#pragma once

// Sets declaration (can be multiple as long as resolve to one definition)

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <units/frequency.h>
#include <units/time.h>

#include <units/length.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>

#include <rev/AbsoluteEncoder.h>
#include <rev/SparkMax.h>
#include <rev/config/ClosedLoopConfig.h>
#include <rev/config/SparkMaxConfig.h>

#include <pathplanner/lib/config/PIDConstants.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/PneumaticsModuleType.h>

#include <math.h>
#include <numbers>

namespace DeviceProperties {
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetSparkMaxConfig();
// Default motor type used for TalonFX motors
extern ctre::phoenix6::configs::TalonFXConfiguration GetTalonFXConfig();
// Default motor type enum for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
// Invert absolute encoder to match direction of motor movement
constexpr bool kInvertEncoder = true;
namespace SystemControl {
// Swerve Kinematics (in cpp)
extern frc::SwerveDriveKinematics<4> kDriveKinematics;
}
}

namespace Mechanism {
// How "tall" the base of the robot is
constexpr units::inch_t kWheelBase = 30_in;
// How wide the base of the robot is
constexpr units::inch_t kTrackWidth = 30_in;
// 990 motor teeth to 195 wheel teeth, converts motor rotations to wheel
// rotations
constexpr units::scalar_t kDriveGearRatio{990.0 / 195.0};
// 1 rot to 2 pi radians, converts motor rotations to radians
constexpr units::scalar_t kAngleGearRatio{2 * std::numbers::pi};
// Max 108 rotations per sec from driving motor, without gear ratios
constexpr units::turns_per_second_t kDriveRps{108};
// Wheel diameter in inches, 3
constexpr units::inch_t kWheelDiameter{3};
// Wheel circumference in meters, ~0.24
constexpr units::meter_t kWheelCircumference{kWheelDiameter * std::numbers::pi};
// Converts meters to rotations, mps to rps, and mps^2 to rps^2, rotations =
// (meterTarget * gearRatio) / WheelCircumferenceMeters
constexpr units::unit_t<units::compound_unit<units::length::meters, units::inverse<units::angle::turns>>> kWheelTurnsToMetersDistance{
    kWheelCircumference / (units::turn_t{kDriveGearRatio.value()})};
// Min voltage required for driving motor to begin moving
constexpr units::volt_t kStaticVoltage{0.15};
// kV for feedforward, target rotation is multipled kV and added to velocity
// control
constexpr units::volt_t kVelocityVoltage{12 / kDriveRps.value()};
// Max speed the wheel move, used to normialize swerve modules speeds to
// maintain control
constexpr units::meters_per_second_t kPhysicalMoveMax{kDriveRps * kWheelCircumference / (units::turn_t{kDriveGearRatio.value()})};
// Nm divided by N resulting in meters of the newton force of the wheels
constexpr units::meter_t kDriveMotorNewtonForce = (kWheelDiameter / 2) / kDriveGearRatio;
}

namespace SysId {
using ramp_rate_t = units::unit_t<units::compound_unit<units::volt, units::inverse<units::second>>>;
namespace Translation {
constexpr ramp_rate_t kRampRate{1.0};
constexpr units::volt_t kStepVoltage{7.0};
constexpr units::second_t kTimeout{10.0};
}
namespace Rotation {
constexpr units::radians_per_second_squared_t kRotationRate{std::numbers::pi / 6.0};
constexpr ramp_rate_t kRampRate = kRotationRate * (1_V * 1_s / 1_rad);
constexpr units::radians_per_second_t kStepRotation{std::numbers::pi};
constexpr units::volt_t kStepVoltage = kStepRotation * (1_V * 1_s / 1_rad);
constexpr units::second_t kTimeout{10.0};
}
namespace Steer {
constexpr ramp_rate_t kRampRate{1.0};
constexpr units::volt_t kStepVoltage{7.0};
constexpr units::second_t kTimeout{10.0};
}

}

namespace Vision {
/* rPi code at https://github.com/VikingsRobotics/PiVision */
constexpr units::second_t kProcessDataEvery = 0.2_s;
constexpr uint8_t kProcessDataOnNth = 16;
constexpr float kRequiredConfidence = 0.0;
constexpr units::meter_t kRequiredDeltaDistance = 3_ft;
constexpr size_t kNumAprilTags = 22;
constexpr frc::Transform3d kCameraMountingPosition{frc::Translation3d{10_in, 0_in, 16_in}, frc::Rotation3d{0_rad, 0_deg, 0_rad}};
}

namespace AutoSettings {
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
}

// Converting java to C++
// Source:
// https://github.com/ElectroBunny/BetaBot2025/blob/develop/src/main/java/frc/robot/commands/AlignToReefTagRelative.java
namespace Align {
// Source:
// https://github.com/ElectroBunny/BetaBot2025/blob/develop/src/main/java/frc/robot/Constants.java
namespace System {
constexpr double kTranslationP = 0.33;
constexpr double kRotationP = 0.33;
}
namespace Location {
constexpr frc::Transform2d kRightBranch{frc::Translation2d{0.5_in, 3.5_in}, frc::Rotation2d{180_deg}};
constexpr frc::Transform2d kLeftBranch{frc::Translation2d{0.5_in, -3.5_in}, frc::Rotation2d{180_deg}};
constexpr frc::Transform2d kBranchThreshold{frc::Translation2d{0.2_in, 0.7_in}, frc::Rotation2d{1_deg}};
}
namespace Time {
constexpr units::second_t kTagOutOfViewTime = 1_s;
constexpr units::second_t kPoseValidation = 0.3_s;
}
}
}

namespace Arm {

namespace TeleopOperator {
// Debounce on different positions
constexpr units::second_t kDebounce{0.1};
// Minimum percent of controller distance before robot response
constexpr double kArmDeadband = 0.15;
}

namespace Destination {
constexpr units::turn_t kAllowableError = 10_deg;
constexpr units::turn_t kAllowableErrorCommand = 15_deg;
constexpr units::second_t kAllowableSwitchTime{0.5};
constexpr units::turn_t kMinTurn = -100.0_deg;
constexpr units::turn_t kMaxTurn = 180.0_deg;
constexpr units::turn_t kBottomTurn = -90_deg;
constexpr units::turn_t kMiddleTurn = 15_deg;
constexpr units::turn_t kTopTurn = 130_deg;
constexpr units::turn_t kCollectTurn = -90_deg;
}

namespace DeviceProperties {
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetSparkMaxConfig();
// Default motor type enum for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
// Invert absolute encoder to match direction of motor movement
constexpr bool kInvertEncoder = false;
}

namespace Mechanism {
constexpr units::volt_t kStaticVoltage{0.0};
constexpr units::volt_t kGravity{0.0};
constexpr units::scalar_t kReducedGearRatio{1.0 / 45.0};
constexpr units::scalar_t kTeethRatio{16.0 / 26.0};
constexpr units::scalar_t kGearRatio = kReducedGearRatio * kTeethRatio;
constexpr units::turn_t kRotationalOffset = -90_deg;
constexpr units::turns_per_second_t kMaxAngularSpeed{0.5};
constexpr units::revolutions_per_minute_t kMaxSpeed = kMaxAngularSpeed / kGearRatio;
constexpr units::turns_per_second_squared_t kMaxAngularAcceleration{0.25};
constexpr units::revolutions_per_minute_per_second_t kMaxAccel = kMaxAngularAcceleration / kGearRatio;
constexpr units::meter_t kArmLength = units::inch_t{16};
constexpr units::meter_t kRequiredClearance = units::inch_t{3};
}
}

namespace Elevator {

namespace ControllerPorts {
// USB ID for xbox controller for driver
constexpr int kDriverControllerPort = 1;
}

namespace TeleopOperator {
// Debounce on different positions
constexpr units::second_t kDebounce{0.1};
// Minimum percent of controller distance before robot response
constexpr double kDriveDeadband = 0.15;
}

namespace Destination {
constexpr units::meter_t kAllowableError = 0.05_in;
constexpr units::meter_t kAllowableErrorCommand = 0.10_in;
constexpr units::second_t kAllowableSwitchTime{0.5};
constexpr units::meter_t kMaxHeight = 24_in;
constexpr units::meter_t kFourthGoal = 23_in;
constexpr units::meter_t kThirdGoal = 18.5_in;
constexpr units::meter_t kSecondGoal = 4.5_in;
// constexpr units::meter_t kFirstGoal = 10_in;
constexpr units::meter_t kCollectionHeight = 7_in;
constexpr units::meter_t kAbsorbCoralHeight = 0.5_in;
constexpr units::meter_t kMinHeight = 0_in;
}

namespace Mechanism {
constexpr units::volt_t kStaticVoltage{0.0};
constexpr units::volt_t kGravity{0.0};
constexpr units::scalar_t kGearRatio{1 / 20.0};
constexpr units::meter_t kGearDiameter = units::inch_t{1.75};
constexpr units::meter_t kGearCircumference = kGearDiameter * std::numbers::pi;
constexpr units::unit_t<units::compound_unit<units::meter, units::inverse<units::turn>>> kRotationToDistance = (kGearCircumference * kGearRatio) / 1_tr;
// 9 inch per second
constexpr units::meters_per_second_t kMaxSpeedInMeters = 1.5_fps;
constexpr units::revolutions_per_minute_t kMaxSpeed = kMaxSpeedInMeters / kRotationToDistance;
// 9 inch per second squared
constexpr units::meters_per_second_squared_t kMaxAccelInMeters = 2_fps_sq;
constexpr units::revolutions_per_minute_per_second_t kMaxAccel = kMaxAccelInMeters / kRotationToDistance;
}

namespace DeviceProperties {
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetElevatorConfig();
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetFollowerConfig();
// Default motor type enum for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
}
}

namespace Roller {
namespace TeleopOperator {
// Debounce on different positions
constexpr units::second_t kDebounce{0.1};
// Minimum percent of controller distance before robot response
constexpr double kDriveDeadband = 0.50;
}
namespace DeviceProperties {
// Pneumatics Hub Type
constexpr frc::PneumaticsModuleType kModuleType = frc::PneumaticsModuleType::CTREPCM;
}
namespace SolenoidId {
constexpr int kForwardChannelId = 1;
constexpr int kReverseChannelId = 0;
}

}

namespace DeviceIdentifier {
// CTRE: CANBus Name for contructors of CRTE software classes
constexpr ctre::phoenix6::CANBus kCANBus{""};
// CTRE: PDH
constexpr int kPDHId = 1;
// REV: Pneumatic Hub
constexpr int kPneumaticHubId = 0;
// CTRE: CANBus Pigeon2 ID
constexpr int kGyroId = 3;
// CTRE: Falcon 500 Front Left Motor ID
constexpr int kFLDriveMotorId = 4;
// REV: Neo 550 Front Left Angle Motor ID
constexpr int kFLAngleMotorId = 5;
// CTRE: Falcon 500 Front Right Motor ID
constexpr int kFRDriveMotorId = 6;
// REV: Neo 550 Front Right Angle Motor ID
constexpr int kFRAngleMotorId = 7;
// CTRE: Falcon 500 Back Left Motor ID
constexpr int kBLDriveMotorId = 8;
// REV: Neo 550 Back Left Angle Motor ID
constexpr int kBLAngleMotorId = 9;
// CTRE: Falcon 500 Back Right Motor ID
constexpr int kBRDriveMotorId = 10;
// REV: Neo 550 Back Right Angle Motor ID
constexpr int kBRAngleMotorId = 11;
// REV: Neo 500 opposite of light
constexpr int kElevatorDriverId = 12;
// REV: Neo 500 by light
constexpr int kElevatorFollowId = 13;
// REV: Neo 500
constexpr int kDirectionMotorId = 14;
// VictorSPX
constexpr int kRepellerWheelId = 15;
// TalonSRX
constexpr int kRollerWheelId = 16;
}
