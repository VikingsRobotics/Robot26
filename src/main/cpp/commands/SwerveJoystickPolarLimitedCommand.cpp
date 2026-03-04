#include "commands/SwerveJoystickPolarLimitedCommand.hpp"

#include "Constants.hpp"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/Smartdashboard.h>

#include <frc2/command/RunCommand.h>

#include "swerve/SwerveRequest.hpp"

SwerveJoystickPolarLimitedCommand::SwerveJoystickPolarLimitedCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick)
    : m_subsystem{subsystem},
      m_joystick{joystick.GetHID()},
      m_limiterMag{Swerve::TeleopOperator::kMagLimiter},
      m_limiterDir{Swerve::TeleopOperator::kDirLimiter},
      m_limiterA{Swerve::TeleopOperator::kAngleLimiter} {
    AddRequirements(m_subsystem);
    SetName("Swerve Joystick Command");
}

void SwerveJoystickPolarLimitedCommand::Initialize() {
    m_fieldCentric = true;
    m_limiterMag.Reset(0);
    m_limiterDir.Reset(0);
    m_limiterA.Reset(0);
}

void SwerveJoystickPolarLimitedCommand::Execute() {
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

    double xIn = m_joystick.GetRawAxis(1);
    double yIn = m_joystick.GetRawAxis(0);
    double mag = std::hypot(xIn, yIn);
    double dir = std::atan2(xIn, yIn);

    double controllerMag = m_limiterMag.Calculate(mag);
    double controllerDir = m_limiterDir.Calculate(dir);
    double controllerRot = m_limiterA.Calculate(m_joystick.GetRawAxis(2));

    double deadbandMove = Swerve::TeleopOperator::kDriveDeadband;
    double deadbandRot = Swerve::TeleopOperator::kDriveAngleDeadband;

    double renormalizedMag = 0.0;
    double renormalizedA = 0.0;

    double normalizedDeadbandMag = controllerMag < 0 ? deadbandMove : -deadbandMove;
    double normalizedDeadbandA = controllerRot < 0 ? deadbandRot : -deadbandRot;

    if (controllerMag > deadbandMove) {
        renormalizedMag = -(controllerMag + normalizedDeadbandMag) / (1 - deadbandMove);
    }
    if (std::abs(controllerRot) > deadbandRot) {
        renormalizedA = -(controllerRot + normalizedDeadbandA) / (1 - deadbandRot);
    }

    units::meters_per_second_t vx = 0_mps;
    units::meters_per_second_t vy = 0_mps;
    units::radians_per_second_t va = 0_rad_per_s;

    vx = (std::cos(controllerDir) * renormalizedMag) * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;
    vy = (std::sin(controllerDir) * renormalizedMag) * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;
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

void SwerveJoystickPolarLimitedCommand::End(bool interrupted) {
    m_subsystem->SetControl(SwerveDriveBrake{});
}

bool SwerveJoystickPolarLimitedCommand::IsFinished() {
    return false;
}