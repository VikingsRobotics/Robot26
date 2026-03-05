#pragma once
#ifndef SUBSYSTEM_SWERVE_H
#define SUBSYSTEM_SWERVE_H

#include <vector>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/util/Color8Bit.h>

#include <frc2/command/SubsystemBase.h>

#include <networktables/NetworkTable.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>


#include "swerve/SwerveDrivetrain.hpp"


class SwerveSubsystem : public frc2::SubsystemBase, public SwerveDrivetrain {
public:
    SwerveSubsystem();
    SwerveSubsystem(SwerveSubsystem& rhs) = delete;
    SwerveSubsystem& operator=(SwerveSubsystem& rhs) = delete;
    SwerveSubsystem(SwerveSubsystem&& rhs) = delete;
    SwerveSubsystem& operator=(SwerveSubsystem&& rhs) = delete;

    void Periodic() override;

public:
    using SwerveDrivetrain::AddVisionMeasurement;
    using SwerveDrivetrain::GetKinematics;
    using SwerveDrivetrain::GetModule;
    using SwerveDrivetrain::GetModuleLocations;
    using SwerveDrivetrain::GetModules;
    using SwerveDrivetrain::GetOdometryFrequency;
    using SwerveDrivetrain::GetOdometryThread;
    using SwerveDrivetrain::GetOperatorForwardDirection;
    using SwerveDrivetrain::GetPigeon2;
    using SwerveDrivetrain::GetState;
    using SwerveDrivetrain::IsOdometryValid;
    using SwerveDrivetrain::RegisterTelemetry;
    using SwerveDrivetrain::ResetPose;
    using SwerveDrivetrain::ResetRotation;
    using SwerveDrivetrain::ResetTranslation;
    using SwerveDrivetrain::RunTempRequest;
    using SwerveDrivetrain::SamplePoseAt;
    using SwerveDrivetrain::SeedFieldCentric;
    using SwerveDrivetrain::SetControl;
    using SwerveDrivetrain::SetOperatorPerspectiveForward;
    using SwerveDrivetrain::SetStateStdDevs;
    using SwerveDrivetrain::SetVisionMeasurementStdDevs;
    using SwerveDrivetrain::TareEverything;

private:
    void Telemeterize(SwerveDriveState const& state);

private:
    frc::Field2d m_field{};
    frc::Field2d m_pose{};

private:
    units::meters_per_second_t MaxSpeed;


    // What to publish over networktables for telemetry
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    // Robot swerve drive state
    std::shared_ptr<nt::NetworkTable> driveStateTable = inst.GetTable("DriveState");
    nt::StructPublisher<frc::Pose3d> drivePose = driveStateTable->GetStructTopic<frc::Pose3d>("Pose").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> driveSpeeds = driveStateTable->GetStructTopic<frc::ChassisSpeeds>("Speeds").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> driveModuleStates = driveStateTable->GetStructArrayTopic<frc::SwerveModuleState>("ModuleStates").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> driveModuleTargets =
        driveStateTable->GetStructArrayTopic<frc::SwerveModuleState>("ModuleTargets").Publish();
    nt::StructArrayPublisher<frc::SwerveModulePosition> driveModulePositions =
        driveStateTable->GetStructArrayTopic<frc::SwerveModulePosition>("ModulePositions").Publish();
    nt::DoublePublisher driveTimestamp = driveStateTable->GetDoubleTopic("Timestamp").Publish();
    nt::DoublePublisher driveOdometryFrequency = driveStateTable->GetDoubleTopic("OdometryFrequency").Publish();

    // Mechanisms to represent the swerve module states
    std::array<frc::Mechanism2d, 4> m_moduleMechanisms{
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
    };

    // A direction and length changing ligament for speed representation
    std::array<frc::MechanismLigament2d*, 4> m_moduleSpeeds{
        m_moduleMechanisms[0].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[1].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[2].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[3].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
    };

    // A direction changing and length constant ligament for module direction
    std::array<frc::MechanismLigament2d*, 4> m_moduleDirections{
        m_moduleMechanisms[0]
            .GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[1]
            .GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[2]
            .GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[3]
            .GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
    };
};

#endif
