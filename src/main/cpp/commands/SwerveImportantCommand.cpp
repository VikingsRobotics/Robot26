#include "commands/SwerveImportantCommand.hpp"

SwerveImportantCommand::SwerveImportantCommand(SwerveSubsystem* const subsystem) : m_subsystem{subsystem}, m_important{"output.chrp"} {
    AddRequirements(m_subsystem);
    SetName("Important Command");
    m_important.AddInstrument(m_subsystem->m_backLeft.m_drivingTalonFx);
    m_important.AddInstrument(m_subsystem->m_backRight.m_drivingTalonFx);
    m_important.AddInstrument(m_subsystem->m_frontLeft.m_drivingTalonFx);
    m_important.AddInstrument(m_subsystem->m_frontRight.m_drivingTalonFx);
    m_important.AddInstrument(m_subsystem->m_gryo);
}

void SwerveImportantCommand::Initialize() {
    m_subsystem->StopModules();
    m_important.Play();
}

void SwerveImportantCommand::Execute() {}

void SwerveImportantCommand::End(bool interrupted) {
    m_important.Stop();
}

bool SwerveImportantCommand::IsFinished() {
    return !m_important.IsPlaying();
}