#include "subsystems/SwerveSubsystem.hpp"

#include <frc/ComputerVisionUtil.h>

#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/util/FlippingUtil.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <units/math.h>

#include <fmt/format.h>

SwerveSubsystem::SwerveSubsystem() : m_getGyroYaw{m_gyro.GetYaw().AsSupplier()} {
    m_gyro.Reset();
    SetName("Swerve Subsystem");

    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configure(
        [this]() { return GetPose2d(); }, [this](frc::Pose2d pose) { ResetOdometry(pose); }, [this]() { return GetCurrentSpeeds(); },
        [this](const frc::ChassisSpeeds speeds, const pathplanner::DriveFeedforwards dff) {
            Drive(speeds, dff.robotRelativeForcesX, dff.robotRelativeForcesY);
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(Swerve::Auto::kTranslationPID, Swerve::Auto::kRotationalPID), config,
        []() {
            return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kRed;
        },
        this);
    frc::SmartDashboard::PutData("Odometry Field", &m_pose);
    frc::SmartDashboard::PutData("Pathplanner Field", &m_field);

    // Logging callback for current robot pose
    pathplanner::PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose) {
        // Do whatever you want with the pose here
        m_field.SetRobotPose(pose);
    });

    // Logging callback for target robot pose
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose) {
        // Do whatever you want with the pose here
        m_field.GetObject("target pose")->SetPose(pose);
    });

    // Logging callback for the active path, this is sent as a vector of poses
    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) {
        // Do whatever you want with the poses here
        m_field.GetObject("path")->SetPoses(poses);
    });

    frc::SmartDashboard::PutData(this);
}

void SwerveSubsystem::Periodic() {
    // Tracks robot position using the position of swerve modules and gryo
    // rotation
    m_pose.SetRobotPose(
        m_poseEstimator.Update(GetRotation2d(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()}));
    frc::SmartDashboard::PutNumber("Gyro (deg)", GetHeading().value());
}

void SwerveSubsystem::ZeroHeading() {
    m_gyro.Reset();
}

void SwerveSubsystem::SetOperatorPerspective(units::turn_t perspective) {
    m_operatorPerspective = perspective;
}

static units::degree_t WrapAngle(units::degree_t angle) {
    angle = units::math::fmod(angle + 180_deg, 360_deg);
    if (angle < 0_deg) angle += 360_deg;
    return angle - 180_deg;
}

units::degree_t SwerveSubsystem::GetHeading() {
    return WrapAngle(m_getGyroYaw());
}

units::degree_t SwerveSubsystem::GetOperatorHeading() {
    return WrapAngle(m_getGyroYaw() + m_operatorPerspective);
}

frc::Rotation2d SwerveSubsystem::GetRotation2d() {
    return frc::Rotation2d{GetHeading()};
}

frc::Pose2d SwerveSubsystem::GetPose2d() {
    return m_poseEstimator.GetEstimatedPosition();
}

void SwerveSubsystem::ResetOdometry(frc::Pose2d pose) {
    // Resets pose but still requires the current state of swerve module and gryo
    // rotation
    m_poseEstimator.ResetPosition(GetRotation2d(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()},
                                  pose);
    m_field.SetRobotPose(GetPose2d());
}

void SwerveSubsystem::StopModules() {
    // Calls every swerve modules Stop function
    SwerveModuleControl stopCommand = SwerveModuleControl{}.WithAzimuthBrake().WithDriveBrake();
    m_frontLeft.ApplyControl(stopCommand);
    m_frontRight.ApplyControl(stopCommand);
    m_backLeft.ApplyControl(stopCommand);
    m_backRight.ApplyControl(stopCommand);
}

void SwerveSubsystem::SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit) {
    // Make sure that we are under max speed
    m_driveKinematics.DesaturateWheelSpeeds(&states, Swerve::Mechanism::kMaxMovement);
    // Calls every swerve modules SetStates function
    m_frontLeft.ApplyControl(m_frontLeft.GetControl(states[0], cosineLimit).WithDriveClosedLoop().WithDriveVoltageMode().WithAzimuthPositionMode());
    m_frontRight.ApplyControl(m_frontRight.GetControl(states[1], cosineLimit).WithDriveClosedLoop().WithDriveVoltageMode().WithAzimuthPositionMode());
    m_backLeft.ApplyControl(m_backLeft.GetControl(states[2], cosineLimit).WithDriveClosedLoop().WithDriveVoltageMode().WithAzimuthPositionMode());
    m_backRight.ApplyControl(m_backRight.GetControl(states[3], cosineLimit).WithDriveClosedLoop().WithDriveVoltageMode().WithAzimuthPositionMode());
}

void SwerveSubsystem::SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit, const std::vector<units::newton_t>& feedforwardX,
                                      const std::vector<units::newton_t>& feedforwardY) {
    // Make sure that we are under max speed
    m_driveKinematics.DesaturateWheelSpeeds(&states, Swerve::Mechanism::kMaxMovement);
    // Calls every swerve modules SetStates function
    m_frontLeft.ApplyControl(m_frontLeft.GetControl(states[0], cosineLimit)
                                 .WithDriveAxisFeedforward(feedforwardX[0], feedforwardY[0])
                                 .WithDriveClosedLoop()
                                 .WithDriveVoltageMode()
                                 .WithAzimuthPositionMode());
    m_frontRight.ApplyControl(m_frontRight.GetControl(states[1], cosineLimit)
                                  .WithDriveAxisFeedforward(feedforwardX[1], feedforwardY[1])
                                  .WithDriveClosedLoop()
                                  .WithDriveVoltageMode()
                                  .WithAzimuthPositionMode());
    m_backLeft.ApplyControl(m_backLeft.GetControl(states[2], cosineLimit)
                                .WithDriveAxisFeedforward(feedforwardX[2], feedforwardY[2])
                                .WithDriveClosedLoop()
                                .WithDriveVoltageMode()
                                .WithAzimuthPositionMode());
    m_backRight.ApplyControl(m_backRight.GetControl(states[3], cosineLimit)
                                 .WithDriveAxisFeedforward(feedforwardX[3], feedforwardY[3])
                                 .WithDriveClosedLoop()
                                 .WithDriveVoltageMode()
                                 .WithAzimuthPositionMode());
}

void SwerveSubsystem::SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit, const std::vector<units::newton_t>& linear) {
    // Make sure that we are under max speed
    m_driveKinematics.DesaturateWheelSpeeds(&states, Swerve::Mechanism::kMaxMovement);
    // Calls every swerve modules SetStates function
    m_frontLeft.ApplyControl(m_frontLeft.GetControl(states[0], cosineLimit)
                                 .WithDriveLinearFeedforward(linear[0])
                                 .WithDriveClosedLoop()
                                 .WithDriveVoltageMode()
                                 .WithAzimuthPositionMode());
    m_frontRight.ApplyControl(m_frontRight.GetControl(states[1], cosineLimit)
                                  .WithDriveLinearFeedforward(linear[1])
                                  .WithDriveClosedLoop()
                                  .WithDriveVoltageMode()
                                  .WithAzimuthPositionMode());
    m_backLeft.ApplyControl(m_backLeft.GetControl(states[2], cosineLimit)
                                .WithDriveLinearFeedforward(linear[2])
                                .WithDriveClosedLoop()
                                .WithDriveVoltageMode()
                                .WithAzimuthPositionMode());
    m_backRight.ApplyControl(m_backRight.GetControl(states[3], cosineLimit)
                                 .WithDriveLinearFeedforward(linear[3])
                                 .WithDriveClosedLoop()
                                 .WithDriveVoltageMode()
                                 .WithAzimuthPositionMode());
}

void SwerveSubsystem::SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit,
                                      const std::vector<units::meters_per_second_squared_t>& accel) {
    // Make sure that we are under max speed
    m_driveKinematics.DesaturateWheelSpeeds(&states, Swerve::Mechanism::kMaxMovement);
    // Calls every swerve modules SetStates function
    m_frontLeft.ApplyControl(m_frontLeft.GetControl(states[0], cosineLimit)
                                 .WithDriveAccelerationFeedforward(accel[0])
                                 .WithDriveClosedLoop()
                                 .WithDriveVoltageMode()
                                 .WithAzimuthPositionMode());
    m_frontRight.ApplyControl(m_frontRight.GetControl(states[1], cosineLimit)
                                  .WithDriveAccelerationFeedforward(accel[1])
                                  .WithDriveClosedLoop()
                                  .WithDriveVoltageMode()
                                  .WithAzimuthPositionMode());
    m_backLeft.ApplyControl(m_backLeft.GetControl(states[2], cosineLimit)
                                .WithDriveAccelerationFeedforward(accel[2])
                                .WithDriveClosedLoop()
                                .WithDriveVoltageMode()
                                .WithAzimuthPositionMode());
    m_backRight.ApplyControl(m_backRight.GetControl(states[3], cosineLimit)
                                 .WithDriveAccelerationFeedforward(accel[3])
                                 .WithDriveClosedLoop()
                                 .WithDriveVoltageMode()
                                 .WithAzimuthPositionMode());
}
frc::ChassisSpeeds SwerveSubsystem::GetCurrentSpeeds() {
    return m_driveKinematics.ToChassisSpeeds({m_frontLeft.GetState(), m_frontRight.GetState(), m_backLeft.GetState(), m_backRight.GetState()});
}

void SwerveSubsystem::Drive(frc::ChassisSpeeds speed) {
    wpi::array<frc::SwerveModuleState, 4> states = m_driveKinematics.ToSwerveModuleStates(speed);
    SetModulesState(states, true);
}

void SwerveSubsystem::Drive(frc::ChassisSpeeds speed, const std::vector<units::newton_t>& feedforwardX, const std::vector<units::newton_t>& feedforwardY) {
    wpi::array<frc::SwerveModuleState, 4> states = m_driveKinematics.ToSwerveModuleStates(speed);
    SetModulesState(states, false, feedforwardX, feedforwardY);
}

void SwerveSubsystem::X() {
    m_frontLeft.ApplyControl(SwerveModuleControl{}.WithDriveBrake().WithAzimuthPosition(45_deg));
    m_frontRight.ApplyControl(SwerveModuleControl{}.WithDriveBrake().WithAzimuthPosition(-45_deg));
    m_backLeft.ApplyControl(SwerveModuleControl{}.WithDriveBrake().WithAzimuthPosition(45_deg));
    m_backRight.ApplyControl(SwerveModuleControl{}.WithDriveBrake().WithAzimuthPosition(-45_deg));
}

ctre::phoenix6::hardware::Pigeon2& SwerveSubsystem::GetGyro() {
    return m_gyro;
}

SwerveModule& SwerveSubsystem::GetFrontLeftModule() {
    return m_frontLeft;
}

SwerveModule& SwerveSubsystem::GetFrontRightModule() {
    return m_frontRight;
}

SwerveModule& SwerveSubsystem::GetBackLeftModule() {
    return m_backLeft;
}

SwerveModule& SwerveSubsystem::GetBackRightModule() {
    return m_backRight;
}

frc::SwerveDrivePoseEstimator<4>& SwerveSubsystem::GetPoseEstimator() {
    return m_poseEstimator;
}

frc::SwerveDriveKinematics<4>& SwerveSubsystem::GetKinematics() {
    return m_driveKinematics;
}