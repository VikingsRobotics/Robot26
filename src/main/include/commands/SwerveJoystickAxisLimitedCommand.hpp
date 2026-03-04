#ifndef COMMAND_SWERVE_AXIS_JOYSTICK_H
#define COMMAND_SWERVE_AXIS_JOYSTICK_H
#pragma once

#include "subsystems/SwerveSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandJoystick.h>

#include <frc/filter/SlewRateLimiter.h>

#include <wpi/array.h>

class SwerveJoystickAxisLimitedCommand : public frc2::CommandHelper<frc2::Command, SwerveJoystickAxisLimitedCommand> {
public:
    SwerveJoystickAxisLimitedCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    SwerveSubsystem* const m_subsystem;
    frc::Joystick& m_joystick;
    bool m_fieldCentric = true;
    frc::SlewRateLimiter<units::scalar> m_limiterX;
    frc::SlewRateLimiter<units::scalar> m_limiterY;
    frc::SlewRateLimiter<units::scalar> m_limiterA;
};

#endif