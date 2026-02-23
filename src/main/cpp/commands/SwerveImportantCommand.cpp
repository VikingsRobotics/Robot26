#include "commands/SwerveImportantCommand.hpp"

SwerveImportantCommand::SwerveImportantCommand(SwerveSubsystem* const subsystem) : m_subsystem{subsystem}, m_important{"output.chrp"} {
    AddRequirements(m_subsystem);
    SetName("Important Command");
    m_important.AddInstrument(m_subsystem->GetFrontLeftModule().GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetFrontRightModule().GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetBackLeftModule().GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetBackRightModule().GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetGyro());
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