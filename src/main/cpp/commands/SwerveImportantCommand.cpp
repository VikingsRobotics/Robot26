#include "commands/SwerveImportantCommand.hpp"
#include "swerve/SwerveRequest.hpp"

SwerveImportantCommand::SwerveImportantCommand(SwerveSubsystem* const subsystem) : m_subsystem{subsystem}, m_important{"output.chrp"} {
    AddRequirements(m_subsystem);
    SetName("Important Command");
    m_important.AddInstrument(m_subsystem->GetModule(0).GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetModule(1).GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetModule(2).GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetModule(3).GetDriveMotor());
    m_important.AddInstrument(m_subsystem->GetPigeon2());
}

void SwerveImportantCommand::Initialize() {
    m_subsystem->SetControl(SwerveDriveBrake{});
    m_important.Play();
}

void SwerveImportantCommand::Execute() {}

void SwerveImportantCommand::End(bool interrupted) {
    m_important.Stop();
}

bool SwerveImportantCommand::IsFinished() {
    return !m_important.IsPlaying();
}