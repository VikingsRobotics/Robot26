#ifndef CONSTANTS_H
#define CONSTANTS_H
#pragma once

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

#include "swerve/SwerveDrivetrainConstants.hpp"
#include "swerve/SwerveModuleConstants.hpp"

namespace DeviceIdentifier {
// CTRE: CANBus Name for contructors of CRTE software classes
constexpr ctre::phoenix6::CANBus kCANBus{""};
// REV: PDH
constexpr int kPDHId = 1;
// REV: PCM
constexpr int kPCMId = 2;
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
constexpr SwerveModuleConstants kFrontLeft =
    SwerveModuleConstants{}
        .WithCouplingGearRatio(Mechanism::kCouplingRatio)
        .WithDriveMotorGearRatio(Mechanism::kDriveGearRatio)
        .WithSteerMotorGearRatio(Mechanism::kAzimuthGearRatio)
        .WithWheelRadius(Mechanism::kWheelDiameter / 2)
        .WithEncoderInverted(true)
        .WithDriveMotorInverted(false)
        .WithSteerMotorInverted(false)
        .WithSpeedAt12Volts(Mechanism::kMaxMovement)
        .WithSlipCurrent(800_A)
        .WithDriveMotorGains(ctre::phoenix6::configs::Slot0Configs{}
                                 .WithKP(0.1)
                                 .WithKI(0.0)
                                 .WithKD(0.0)
                                 .WithKS(0.15)
                                 .WithKV(0.00188)
                                 .WithKA(0.0)
                                 .WithKG(0.0)
                                 .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Elevator_Static)
                                 .WithStaticFeedforwardSign(ctre::phoenix6::signals::StaticFeedforwardSignValue::UseVelocitySign))
        .WithSteerMotorGains(SwerveModuleConstants::Slot0ConfigsRev{.kP = 1.0,
                                                                    .kI = 0.0,
                                                                    .kD = 0.0,
                                                                    .dFilter = 0.0,
                                                                    .iZone = 0.0,
                                                                    .iMaxAccum = 0.0,
                                                                    .minOut = -1.0,
                                                                    .maxOut = 1.0,
                                                                    .posWrapEnabled = true,
                                                                    .posMinInput = 0.0,
                                                                    .posMaxInput = 1.0,
                                                                    .sensor = rev::spark::FeedbackSensor::kAbsoluteEncoder,
                                                                    .cruiseVelocity = Mechanism::kMaxRotation(),
                                                                    .maxAcceleration = 3_tr_per_s_sq(),
                                                                    .allowedError = 0.5,
                                                                    .kS = 0.0,
                                                                    .kV = 0.0,
                                                                    .kA = 0.0})
        .WithDriveMotorId(DeviceIdentifier::kFLDriveMotorId)
        .WithSteerMotorId(DeviceIdentifier::kFLAngleMotorId)
        .WithLocationX(+Mechanism::kWheelBase / 2)
        .WithLocationY(+Mechanism::kTrackWidth / 2)
        .WithEncoderOffset(-0.25_tr);
;

constexpr SwerveModuleConstants kFrontRight = SwerveModuleConstants{kFrontLeft}
                                                  .WithDriveMotorId(DeviceIdentifier::kFRDriveMotorId)
                                                  .WithSteerMotorId(DeviceIdentifier::kFRAngleMotorId)
                                                  .WithLocationX(+Mechanism::kWheelBase / 2)
                                                  .WithLocationY(-Mechanism::kTrackWidth / 2)
                                                  .WithEncoderOffset(+0.00_tr);

constexpr SwerveModuleConstants kBackLeft = SwerveModuleConstants{kFrontLeft}
                                                .WithDriveMotorId(DeviceIdentifier::kBLDriveMotorId)
                                                .WithSteerMotorId(DeviceIdentifier::kBLAngleMotorId)
                                                .WithLocationX(-Mechanism::kWheelBase / 2)
                                                .WithLocationY(+Mechanism::kTrackWidth / 2)
                                                .WithEncoderOffset(+0.50_tr);

constexpr SwerveModuleConstants kBackRight = SwerveModuleConstants{kFrontLeft}
                                                 .WithDriveMotorId(DeviceIdentifier::kBRDriveMotorId)
                                                 .WithSteerMotorId(DeviceIdentifier::kBRAngleMotorId)
                                                 .WithLocationX(-Mechanism::kWheelBase / 2)
                                                 .WithLocationY(-Mechanism::kTrackWidth / 2)
                                                 .WithEncoderOffset(+0.25_tr);

constexpr SwerveDrivetrainConstants kDrivetrain =
    SwerveDrivetrainConstants{}.WithPigeon2Id(DeviceIdentifier::kGyroId).WithPigeon2Configs(std::nullopt).WithCANBusName("");

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
// Maximum speed that the robot will move
constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;
// Maximum accel that the robot will move
constexpr units::meters_per_second_squared_t kDriveMoveAccelMax = 3.0_mps_sq;
// Maximum turning speed that the robot will move
constexpr units::turns_per_second_t kDriveAngleSpeedMax = 1_tps;
// Maximum turning accel that the robot will move
constexpr units::turns_per_second_squared_t kDriveAngleAccelMax = 1_tr_per_s_sq;

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
