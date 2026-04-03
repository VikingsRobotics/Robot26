#include "commands/ElevatorControllerCommand.hpp"

ElevatorControllerCommand::ElevatorControllerCommand(ElevatorSubsystem* const subsystem, frc2::CommandXboxController& controller)
    : m_subsystem{subsystem}, m_controller{controller.GetHID()} {
    AddRequirements(m_subsystem);
    SetName("Elevator Controller Command");
}

void ElevatorControllerCommand::Initialize() {}

void ElevatorControllerCommand::Execute() {
    if (m_controller.GetBButtonPressed() && !m_subsystem->IsElevatorUp()) {
        m_subsystem->PutElevatorUp();
    }
    if (m_controller.GetAButtonPressed() && !m_subsystem->IsElevatorDown()) {
        m_subsystem->PutElevatorDown();
    }
}

void ElevatorControllerCommand::End(bool interrupted) {}

bool ElevatorControllerCommand::IsFinished() {
    return false;
}