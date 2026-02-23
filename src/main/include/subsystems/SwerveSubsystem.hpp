#pragma once
#ifndef SUBSYSTEM_SWERVE_H
#define SUBSYSTEM_SWERVE_H

#include "subsystems/SwerveModule.hpp"

#include "Constants.hpp"

#include <optional>
#include <vector>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc/smartdashboard/Field2d.h>

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc2/command/SubsystemBase.h>


class SwerveSubsystem : public frc2::SubsystemBase {
public:
    SwerveSubsystem();
    SwerveSubsystem(SwerveSubsystem& rhs) = delete;
    SwerveSubsystem& operator=(SwerveSubsystem& rhs) = delete;
    SwerveSubsystem(SwerveSubsystem&& rhs) = delete;
    SwerveSubsystem& operator=(SwerveSubsystem&& rhs) = delete;

    void Periodic() override;

    void ZeroHeading();

    void SetOperatorPerspective(units::turn_t perspective);

    units::degree_t GetHeading();

    units::degree_t GetOperatorHeading();
    
    frc::Rotation2d GetRotation2d();

    frc::Pose2d GetPose2d();

    void ResetOdometry(frc::Pose2d pose);

    void StopModules();

    void SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit);


    void SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit, const std::vector<units::newton_t>& feedforwardX,
                         const std::vector<units::newton_t>& feedforwardY);
    void SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit, const std::vector<units::newton_t>& linear);
    void SetModulesState(wpi::array<frc::SwerveModuleState, 4> states, bool cosineLimit, const std::vector<units::meters_per_second_squared_t>& accel);

    frc::ChassisSpeeds GetCurrentSpeeds();

    void Drive(frc::ChassisSpeeds speed);
    void Drive(frc::ChassisSpeeds speed, const std::vector<units::newton_t>& feedforwardX, const std::vector<units::newton_t>& feedforwardY);

    void X();

    ctre::phoenix6::hardware::Pigeon2& GetGyro();
    SwerveModule& GetFrontLeftModule();
    SwerveModule& GetFrontRightModule();
    SwerveModule& GetBackLeftModule();
    SwerveModule& GetBackRightModule();

    frc::SwerveDrivePoseEstimator<4>& GetPoseEstimator();
    frc::SwerveDriveKinematics<4>& GetKinematics();

private:
    // Gryo used for odometry and for field centric control
    ctre::phoenix6::hardware::Pigeon2 m_gyro{DeviceIdentifier::kGyroId, DeviceIdentifier::kCANBus};
    std::function<units::angle::degree_t()> m_getGyroYaw;
    // Front Left module
    SwerveModule m_frontLeft{Swerve::DeviceProperties::kFrontLeft};
    // Front Right module
    SwerveModule m_frontRight{Swerve::DeviceProperties::kFrontRight};
    // Back Left module
    SwerveModule m_backLeft{Swerve::DeviceProperties::kBackLeft};
    // Back Right module
    SwerveModule m_backRight{Swerve::DeviceProperties::kBackRight};
    // Kinematics matrix stuff
    frc::SwerveDriveKinematics<4> m_driveKinematics{Swerve::DeviceProperties::kFrontLeft.kModuleTranslation, Swerve::DeviceProperties::kFrontRight.kModuleTranslation,
                                                       Swerve::DeviceProperties::kBackLeft.kModuleTranslation, Swerve::DeviceProperties::kBackRight.kModuleTranslation};
    // Track the position of the robot using wheel position and gryo rotation
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        m_driveKinematics,
        frc::Rotation2d{},
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()},
        frc::Pose2d{},
        {0.1, 0.1, 0.1},
        {0.1, 0.1, 0.1}};
    frc::Field2d m_field{};
    frc::Field2d m_pose{};

    units::turn_t m_operatorPerspective = 0_tr;
};

#endif
