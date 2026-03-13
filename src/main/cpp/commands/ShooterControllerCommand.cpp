#include "commands/ShooterControllerCommand.hpp"

#include "shooter/ShooterRequest.hpp"
#include "Constants.hpp"

ShooterControllerCommand::ShooterControllerCommand(ShooterSubsystem* const subsystem, frc2::CommandXboxController& controller)
    : m_subsystem{subsystem}, m_controller{controller}, m_limiter{Shooter::TeleopOperator::kFlywheelLimiter} {
    AddRequirements(m_subsystem);
    SetName("Shooter Controller Command");
}

void ShooterControllerCommand::Initialize() {
    m_limiter.Reset(units::dimensionless::scalar_t{0});
}

void ShooterControllerCommand::Execute() {
    double control = m_limiter.Calculate(m_controller.GetRightY());

    double deadband = Shooter::TeleopOperator::kFlywheelDeadband;

    double renormalized = 0.0;

    if (control > deadband) {
        renormalized = -(control + deadband) / (1 - deadband);
    }

    m_subsystem->SetControl(SurfaceRequest{}.WithSpeeds(renormalized * Shooter::TeleopOperator::kFlywheelMaxSpeed).WithType(FlywheelRequestType::kClosedLoop));
}

void ShooterControllerCommand::End(bool interrupted) {
    m_subsystem->SetControl(ShooterBrake{});
}

bool ShooterControllerCommand::IsFinished() {
    return false;
}