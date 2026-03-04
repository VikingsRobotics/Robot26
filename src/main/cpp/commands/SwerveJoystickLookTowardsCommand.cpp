#include "commands/SwerveJoystickLookTowardsCommand.hpp"

#include "Constants.hpp"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/Smartdashboard.h>

#include <frc2/command/RunCommand.h>

#include "swerve/SwerveRequest.hpp"

SwerveJoystickLookTowardsCommand::SwerveJoystickLookTowardsCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick)
    : m_subsystem{subsystem}, m_joystick{joystick.GetHID()} {
    AddRequirements(m_subsystem);
    SetName("Swerve Joystick Command");
}

void SwerveJoystickLookTowardsCommand::Initialize() {
    m_lastRotation = frc::Rotation2d{};
}

void SwerveJoystickLookTowardsCommand::Execute() {
    if (m_joystick.GetRawButtonPressed(5)) {
        m_subsystem->SeedFieldCentric();
    }
    if (m_joystick.GetRawButton(1)) {
        m_subsystem->SetControl(SwerveDriveBrake{});
        return;
    }

    double xIn = m_joystick.GetRawAxis(1);
    double yIn = m_joystick.GetRawAxis(0);
    double mag = std::hypot(xIn, yIn);
    frc::Rotation2d desiredRotation;

    if (mag <= Swerve::TeleopOperator::kDriveDeadband) {
        desiredRotation = m_lastRotation;
    } else {
        desiredRotation = frc::Rotation2d{xIn, yIn};
    }
    m_lastRotation = desiredRotation;

    m_subsystem->SetControl(FieldCentricFacingAngle{}
                                .WithHeadingPID(Swerve::Auto::kRotationalPID.kP, Swerve::Auto::kRotationalPID.kI, Swerve::Auto::kRotationalPID.kD)
                                .WithForwardPerspective(ForwardPerspectiveValue::OperatorPerspective)
                                .WithTargetDirection(desiredRotation)
                                .WithVelocityX(0_mps)
                                .WithVelocityY(0_mps)
                                .WithDriveRequestType(ModuleDriveRequestType::Velocity)
                                .WithSteerRequestType(ModuleSteerRequestType::Position)
                                .WithMaxAbsRotationalRate(Swerve::Auto::kMaxAngularSpeed));
}

void SwerveJoystickLookTowardsCommand::End(bool interrupted) {
    m_subsystem->SetControl(SwerveDriveBrake{});
}

bool SwerveJoystickLookTowardsCommand::IsFinished() {
    return false;
}