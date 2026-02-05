#pragma once
#ifndef COMMAND_SWERVE_IMPORTANT_H
#define COMMAND_SWERVE_IMPORTANT_H

#include "subsystems/SwerveSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <ctre/phoenix6/Orchestra.hpp>

class SwerveImportantCommand : public frc2::CommandHelper<frc2::Command, SwerveImportantCommand> {
public:
    SwerveImportantCommand(SwerveSubsystem* const subsystem);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    SwerveSubsystem* const m_subsystem;
    ctre::phoenix6::Orchestra m_important;
};

#endif