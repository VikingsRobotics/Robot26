#ifndef COMMAND_SHOOTER_CONTROLLER_H
#define COMMAND_SHOOTER_CONTROLLER_H
#pragma once

#include "subsystems/ShooterSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/button/CommandXboxController.h>

class ShooterControllerCommand : public frc2::CommandHelper<frc2::Command, ShooterControllerCommand> {
public:
    ShooterControllerCommand(ShooterSubsystem* const subsystem, frc2::CommandXboxController& controller);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    ShooterSubsystem* const m_subsystem;
    frc2::CommandXboxController& m_controller;
    frc::SlewRateLimiter<units::scalar> m_limiter;
    bool running = false;
    units::second_t timestampStart = 0_s;
};

#endif