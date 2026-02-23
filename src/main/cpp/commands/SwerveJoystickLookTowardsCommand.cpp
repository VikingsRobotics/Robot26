#include "commands/SwerveJoystickLookTowardsCommand.hpp"

#include "Constants.hpp"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/Smartdashboard.h>

#include <frc2/command/RunCommand.h>

SwerveJoystickLookTowardsCommand::SwerveJoystickLookTowardsCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick)
    : m_subsystem{subsystem},
      m_joystick{joystick.GetHID()},
      m_rotationPID{
        Swerve::TeleopOperator::kLookPID.kP(),
        Swerve::TeleopOperator::kLookPID.kI(),
        Swerve::TeleopOperator::kLookPID.kD(),
        frc::ProfiledPIDController<units::radians>::Constraints{
            Swerve::TeleopOperator::kDriveAngleSpeedMax,
            Swerve::TeleopOperator::kDriveAngleAccelMax
        }
      } {
    AddRequirements(m_subsystem);
    m_rotationPID.EnableContinuousInput(-0.5_tr,0.5_tr);
    m_rotationPID.SetTolerance(2_deg,5_deg_per_s);
    SetName("Swerve Joystick Command");
}

void SwerveJoystickLookTowardsCommand::Initialize() {
    m_rotationPID.Reset(m_subsystem->GetHeading(),m_subsystem->GetGyro().GetAngularVelocityZDevice().GetValue());
    m_lastRotation = m_subsystem->GetRotation2d();
}

void SwerveJoystickLookTowardsCommand::Execute() {
    if (m_joystick.GetRawButtonPressed(5)) {
        m_subsystem->ZeroHeading();
    }
    if (m_joystick.GetRawButton(1)) {
        m_subsystem->StopModules();
        return;
    }

    double xIn = m_joystick.GetRawAxis(1);
    double yIn = m_joystick.GetRawAxis(0);
    double mag = std::hypot(xIn,yIn);
    frc::Rotation2d desiredRotation;

    if(mag <= Swerve::TeleopOperator::kDriveDeadband)
    {
        desiredRotation = m_lastRotation;
    }
    else
    {
        desiredRotation = frc::Rotation2d{xIn,yIn};
    }
    m_lastRotation = desiredRotation;

    units::radians_per_second_t va { m_rotationPID.Calculate(m_subsystem->GetOperatorHeading(),desiredRotation.Degrees()) };
    
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(0_mps, 0_mps, va, m_subsystem->GetOperatorHeading());

    wpi::array<frc::SwerveModuleState, 4> swerveModule = m_subsystem->GetKinematics().ToSwerveModuleStates(speeds);

    m_subsystem->SetModulesState(swerveModule, false);
}

void SwerveJoystickLookTowardsCommand::End(bool interrupted) {
    m_subsystem->StopModules();
}

bool SwerveJoystickLookTowardsCommand::IsFinished() {
    return false;
}