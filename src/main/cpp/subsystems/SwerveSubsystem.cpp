#include "subsystems/SwerveSubsystem.hpp"

#include "Constants.hpp"
#include "swerve/SwerveRequest.hpp"

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

SwerveSubsystem::SwerveSubsystem()
    : SwerveDrivetrain{Swerve::DeviceProperties::kDrivetrain, std::array<double, 4>{0.9, 0.9, 0.9, 0.9}, std::array<double, 4>{0.1, 0.1, 0.1, 0.1},
                       std::array<SwerveModuleConstants, 4>{Swerve::DeviceProperties::kFrontLeft, Swerve::DeviceProperties::kFrontRight,
                                                            Swerve::DeviceProperties::kBackLeft, Swerve::DeviceProperties::kBackRight}},
      MaxSpeed{Swerve::Mechanism::kMaxMovement} {
    SetName("Swerve Subsystem");

    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configure(
        [this]() { return GetState().Pose.ToPose2d(); }, [this](frc::Pose2d pose) { ResetPose(frc::Pose3d{pose}); }, [this]() { return GetState().Speeds; },
        [this](const frc::ChassisSpeeds speeds, const pathplanner::DriveFeedforwards dff) {
            SetControl(ApplyRobotSpeeds{}
                           .WithSpeeds(speeds)
                           .WithWheelForceFeedforwardsX(dff.robotRelativeForcesX)
                           .WithWheelForceFeedforwardsY(dff.robotRelativeForcesY)
                           .WithDriveRequestType(ModuleDriveRequestType::Velocity)
                           .WithSteerRequestType(ModuleSteerRequestType::Position)
                           .WithDesaturateWheelSpeeds(true));
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(Swerve::Auto::kTranslationPID, Swerve::Auto::kRotationalPID), config,
        []() { return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kRed; }, this);

    frc::SmartDashboard::PutData("Odometry Field", &m_pose);
    frc::SmartDashboard::PutData("Pathplanner Field", &m_field);

    pathplanner::PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose) { m_field.SetRobotPose(pose); });

    pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose) { m_field.GetObject("target pose")->SetPose(pose); });

    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) { m_field.GetObject("path")->SetPoses(poses); });

    RegisterTelemetry([this](SwerveDriveState const& state) { Telemeterize(state); });
}

static units::degree_t WrapAngle(units::degree_t angle) {
    angle = units::math::fmod(angle + 180_deg, 360_deg);
    if (angle < 0_deg) angle += 360_deg;
    return angle - 180_deg;
}

void SwerveSubsystem::AddFunctionCallbackOnTelemerty(std::function<void(SwerveDriveState const&)> callback) {
    RegisterTelemetry([this, callback](SwerveDriveState const& state) {
        Telemeterize(state);
        callback(state);
    });
};

void SwerveSubsystem::Telemeterize(SwerveDriveState const& state) {
    frc::SmartDashboard::PutNumber("Gyro Yaw (deg)", WrapAngle(state.Pose.ToPose2d().Rotation().Degrees())());

    drivePose.Set(state.Pose);
    driveSpeeds.Set(state.Speeds);
    driveModuleStates.Set(state.ModuleStates);
    driveModuleTargets.Set(state.ModuleTargets);
    driveModulePositions.Set(state.ModulePositions);
    driveTimestamp.Set(state.Timestamp.value());
    driveOdometryFrequency.Set(1.0 / state.OdometryPeriod.value());

    m_pose.SetRobotPose(state.Pose.ToPose2d());

    m_moduleDirections[0]->SetAngle(state.ModuleStates[0].angle.Degrees());
    m_moduleSpeeds[0]->SetAngle(state.ModuleStates[0].angle.Degrees());
    m_moduleSpeeds[0]->SetLength(state.ModuleStates[0].speed / (2 * MaxSpeed));
    m_moduleDirections[1]->SetAngle(state.ModuleStates[1].angle.Degrees());
    m_moduleSpeeds[1]->SetAngle(state.ModuleStates[1].angle.Degrees());
    m_moduleSpeeds[1]->SetLength(state.ModuleStates[1].speed / (2 * MaxSpeed));
    m_moduleDirections[2]->SetAngle(state.ModuleStates[2].angle.Degrees());
    m_moduleSpeeds[2]->SetAngle(state.ModuleStates[2].angle.Degrees());
    m_moduleSpeeds[2]->SetLength(state.ModuleStates[2].speed / (2 * MaxSpeed));
    m_moduleDirections[3]->SetAngle(state.ModuleStates[3].angle.Degrees());
    m_moduleSpeeds[3]->SetAngle(state.ModuleStates[3].angle.Degrees());
    m_moduleSpeeds[3]->SetLength(state.ModuleStates[3].speed / (2 * MaxSpeed));
}