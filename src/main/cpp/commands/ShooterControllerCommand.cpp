#include "commands/ShooterControllerCommand.hpp"

#include <frc/Timer.h>

#include "shooter/ShooterRequest.hpp"
#include "Constants.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

ShooterControllerCommand::ShooterControllerCommand(ShooterSubsystem* const subsystem, frc2::CommandXboxController& controller)
    : m_subsystem{subsystem}, m_controller{controller.GetHID()}, m_limiter{Shooter::TeleopOperator::kFlywheelLimiter} {
    AddRequirements(m_subsystem);
    SetName("Shooter Controller Command");
}

void ShooterControllerCommand::Initialize() {
    m_limiter.Reset(units::dimensionless::scalar_t{0});
    running = false;
}

void ShooterControllerCommand::Execute() {
    double control = m_limiter.Calculate(m_controller.GetRightY());

    double deadband = Shooter::TeleopOperator::kFlywheelDeadband;

    double renormalized = 0.0;

    if (control < -deadband) {
        renormalized = -(control + deadband) / (1 - deadband);
    }

    if (renormalized > 0.0) {
        if (!running) {
            timestampStart = frc::Timer::GetTimestamp();
        }
        running = true;
        m_subsystem->SetControl(
            SurfaceRequest{}
                .WithSpeeds(renormalized * Shooter::TeleopOperator::kFlywheelMaxSpeed)
                .WithType(FlywheelRequestType::kClosedLoop)
                .WithFeederSpeed(units::dimensionless::scalar_t{
                    (frc::Timer::GetTimestamp() - timestampStart) > Shooter::TeleopOperator::kTimeout ? Shooter::TeleopOperator::kFeederSpeed() : 0.0}));
        frc::SmartDashboard::PutNumber("Flywheel Speed", (renormalized * Shooter::TeleopOperator::kFlywheelMaxSpeed)());
    } else {
        running = false;
        m_subsystem->SetControl(ShooterBrake{});
    }
    frc::SmartDashboard::PutBoolean("Flywheel Running", running);
}

void ShooterControllerCommand::End(bool interrupted) {
    m_subsystem->SetControl(ShooterBrake{});
}

bool ShooterControllerCommand::IsFinished() {
    return false;
}