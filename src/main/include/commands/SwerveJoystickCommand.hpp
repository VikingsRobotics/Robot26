#pragma once
#ifndef COMMAND_SWERVE_JOYSTICK_H
#define COMMAND_SWERVE_JOYSTICK_H

#include "subsystems/SwerveSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandJoystick.h>

#include <frc/filter/SlewRateLimiter.h>

#include <wpi/array.h>

class SwerveJoystickCommand : public frc2::CommandHelper<frc2::Command, SwerveJoystickCommand> {
public:
    SwerveJoystickCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    SwerveSubsystem* const m_subsystem;
    frc::Joystick& m_joystick;
    wpi::array<frc::Rotation2d, 4> m_lastState{{frc::Rotation2d{}, frc::Rotation2d{}, frc::Rotation2d{}, frc::Rotation2d{}}};
    bool m_fieldCentric = true;
    bool m_precision = false;
    frc::SlewRateLimiter<units::scalar> m_limiterX;
    frc::SlewRateLimiter<units::scalar> m_limiterY;
    frc::SlewRateLimiter<units::scalar> m_limiterA;
};

#endif