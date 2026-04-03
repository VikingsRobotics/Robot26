#ifndef COMMAND_CAGE_CONTROLLER_H
#define COMMAND_CAGE_CONTROLLER_H
#pragma once

#include "subsystems/CageSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandXboxController.h>

class CageControllerCommand : public frc2::CommandHelper<frc2::Command, CageControllerCommand> {
public:
    CageControllerCommand(CageSubsystem* const subsystem, frc2::CommandXboxController& controller);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    CageSubsystem* const m_subsystem;
    frc::XboxController& m_controller;
};

#endif