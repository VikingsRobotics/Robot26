#pragma once
#ifndef COMMAND_SWERVE_POINT_JOYSTICK_H
#define COMMAND_SWERVE_POINT_JOYSTICK_H

#include "subsystems/SwerveSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandJoystick.h>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/controller/ProfiledPIDController.h>

#include <wpi/array.h>

class SwerveJoystickLookTowardsCommand : public frc2::CommandHelper<frc2::Command, SwerveJoystickLookTowardsCommand> {
public:
    SwerveJoystickLookTowardsCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    SwerveSubsystem* const m_subsystem;
    frc::Joystick& m_joystick;
    frc::Rotation2d m_lastRotation;
    frc::ProfiledPIDController<units::radians> m_rotationPID;
};

#endif