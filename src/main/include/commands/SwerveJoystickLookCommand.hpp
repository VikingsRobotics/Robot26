#ifndef COMMAND_SWERVE_POINT_JOYSTICK_H
#define COMMAND_SWERVE_POINT_JOYSTICK_H
#pragma once

#include "subsystems/SwerveSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandJoystick.h>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>

#include <wpi/array.h>

class SwerveJoystickLookCommand : public frc2::CommandHelper<frc2::Command, SwerveJoystickLookCommand> {
public:
    SwerveJoystickLookCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    SwerveSubsystem* const m_subsystem;
    frc::Joystick& m_joystick;
    frc::Rotation2d m_lastRotation;

    bool m_globalLook = false;

    units::second_t m_prevTime = 0_s;

    frc::SlewRateLimiter<units::scalar> m_limiterMag;
    frc::Rotation2d m_currentTranslationDir{};
    units::dimensionless::scalar_t m_currentTranslationMag{0.0};
};

#endif