#include "commands/CageControllerCommand.hpp"

CageControllerCommand::CageControllerCommand(CageSubsystem* const subsystem, frc2::CommandXboxController& controller)
    : m_subsystem{subsystem}, m_controller{controller.GetHID()} {
    AddRequirements(m_subsystem);
    SetName("Cage Controller Command");
}

void CageControllerCommand::Initialize() {}

void CageControllerCommand::Execute() {
    if (m_controller.GetXButtonPressed() && !m_subsystem->IsCageExpanded()) {
        m_subsystem->ExpandCage();
    }
    if (m_controller.GetYButtonPressed() && !m_subsystem->IsCageCompressed()) {
        m_subsystem->CompressCage();
    }
}

void CageControllerCommand::End(bool interrupted) {}

bool CageControllerCommand::IsFinished() {
    return false;
}