#ifndef COMMAND_ELEVATOR_CONTROLLER_H
#define COMMAND_ELEVATOR_CONTROLLER_H
#pragma once

#include "subsystems/ElevatorSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandXboxController.h>

class ElevatorControllerCommand : public frc2::CommandHelper<frc2::Command, ElevatorControllerCommand> {
public:
    ElevatorControllerCommand(ElevatorSubsystem* const subsystem, frc2::CommandXboxController& controller);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    ElevatorSubsystem* const m_subsystem;
    frc::XboxController& m_controller;
};

#endif