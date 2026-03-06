#include "commands/SwerveJoystickAxisLimitedCommand.hpp"

#include "Constants.hpp"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/Smartdashboard.h>

#include <frc2/command/RunCommand.h>

#include "swerve/SwerveRequest.hpp"

SwerveJoystickAxisLimitedCommand::SwerveJoystickAxisLimitedCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick)
    : m_subsystem{subsystem},
      m_joystick{joystick.GetHID()},
      m_limiterX{Swerve::TeleopOperator::kTransLimiter},
      m_limiterY{Swerve::TeleopOperator::kTransLimiter},
      m_limiterA{Swerve::TeleopOperator::kAngleLimiter} {
    AddRequirements(m_subsystem);
    SetName("Swerve Axis Joystick Command");
}

void SwerveJoystickAxisLimitedCommand::Initialize() {
    m_fieldCentric = true;
    m_limiterX.Reset(0);
    m_limiterY.Reset(0);
    m_limiterA.Reset(0);
}

void SwerveJoystickAxisLimitedCommand::Execute() {
    double throttle = (-m_joystick.GetRawAxis(3) + 1) / 2;
    frc::SmartDashboard::PutNumber("Throttle", throttle);
    frc::SmartDashboard::PutBoolean("Field Centric", m_fieldCentric);
    if (m_joystick.GetRawButtonPressed(2)) {
        m_fieldCentric = !m_fieldCentric;
    }
    if (m_joystick.GetRawButtonPressed(5)) {
        m_subsystem->SeedFieldCentric();
    }
    if (m_joystick.GetRawButton(1)) {
        m_subsystem->SetControl(SwerveDriveBrake{});
        return;
    }

    double controllerX = m_limiterX.Calculate(m_joystick.GetRawAxis(1));
    double controllerY = m_limiterY.Calculate(m_joystick.GetRawAxis(0));
    double controllerRot = m_limiterA.Calculate(m_joystick.GetRawAxis(2));

    double deadbandMove = Swerve::TeleopOperator::kDriveDeadband;
    double deadbandRot = Swerve::TeleopOperator::kDriveAngleDeadband;

    double renormalizedX = 0.0;
    double renormalizedY = 0.0;
    double renormalizedA = 0.0;

    double normalizedDeadbandX = controllerX < 0 ? deadbandMove : -deadbandMove;
    double normalizedDeadbandY = controllerY < 0 ? deadbandMove : -deadbandMove;
    double normalizedDeadbandA = controllerRot < 0 ? deadbandRot : -deadbandRot;

    // X^2 + Y^2 > D^2
    // Easier process than sqrt(X^2 + Y^2) > Deadband
    if ((controllerX * controllerX + controllerY * controllerY) > (deadbandMove * deadbandMove)) {
        renormalizedX = -(controllerX + normalizedDeadbandX) / (1 - deadbandMove);
        renormalizedY = -(controllerY + normalizedDeadbandY) / (1 - deadbandMove);
    }
    if (std::abs(controllerRot) > deadbandRot) {
        renormalizedA = -(controllerRot + normalizedDeadbandA) / (1 - deadbandRot);
    }

    units::meters_per_second_t vx = 0_mps;
    units::meters_per_second_t vy = 0_mps;
    units::radians_per_second_t va = 0_rad_per_s;

    vx = renormalizedX * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;
    vy = renormalizedY * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;
    va = renormalizedA * Swerve::TeleopOperator::kDriveAngleSpeedMax;

    if (m_fieldCentric) {
        m_subsystem->SetControl(FieldCentric{}
                                    .WithVelocityX(vx)
                                    .WithVelocityY(vy)
                                    .WithRotationalRate(va)
                                    .WithDriveRequestType(ModuleDriveRequestType::Velocity)
                                    .WithSteerRequestType(ModuleSteerRequestType::Position)
                                    .WithForwardPerspective(ForwardPerspectiveValue::OperatorPerspective));
    } else {
        m_subsystem->SetControl(RobotCentric{}
                                    .WithVelocityX(vx)
                                    .WithVelocityY(vy)
                                    .WithRotationalRate(va)
                                    .WithDriveRequestType(ModuleDriveRequestType::Velocity)
                                    .WithSteerRequestType(ModuleSteerRequestType::Position));
    }
}

void SwerveJoystickAxisLimitedCommand::End(bool interrupted) {
    m_subsystem->SetControl(SwerveDriveBrake{});
}

bool SwerveJoystickAxisLimitedCommand::IsFinished() {
    return false;
}