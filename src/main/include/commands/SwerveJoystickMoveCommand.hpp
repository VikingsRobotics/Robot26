#ifndef COMMAND_SWERVE_AXIS_JOYSTICK_H
#define COMMAND_SWERVE_AXIS_JOYSTICK_H
#pragma once

#include "subsystems/SwerveSubsystem.hpp"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandJoystick.h>

#include <frc/filter/SlewRateLimiter.h>

#include <wpi/array.h>
#include <utility>

#include <frc/geometry/Rotation2d.h>
#include <units/time.h>

class SwerveJoystickMoveCommand : public frc2::CommandHelper<frc2::Command, SwerveJoystickMoveCommand> {
public:
    SwerveJoystickMoveCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    SwerveJoystickMoveCommand& WithDeadband(double deadbandTran, double deadbandMag, double deadbandA) & {
        this->deadbandTran = deadbandTran;
        this->deadbandMag = deadbandMag;
        this->deadbandA = deadbandA;
        return *this;
    }
    SwerveJoystickMoveCommand&& WithDeadband(double deadbandTran, double deadbandMag, double deadbandA) && {
        this->deadbandTran = deadbandTran;
        this->deadbandMag = deadbandMag;
        this->deadbandA = deadbandA;
        return std::move(*this);
    }

    SwerveJoystickMoveCommand& WithFieldCentric(bool fieldCentric) & {
        this->m_fieldCentric = fieldCentric;
        return *this;
    }
    SwerveJoystickMoveCommand&& WithFieldCentric(bool fieldCentric) && {
        this->m_fieldCentric = fieldCentric;
        return std::move(*this);
    }

    SwerveJoystickMoveCommand& WithAxisLimited(bool axisLimited) & {
        this->m_axisLimited = axisLimited;
        return *this;
    }
    SwerveJoystickMoveCommand&& WithAxisLimited(bool axisLimited) && {
        this->m_axisLimited = axisLimited;
        return std::move(*this);
    }

private:
    SwerveSubsystem* const m_subsystem;
    frc::Joystick& m_joystick;

    units::second_t m_prevTime = 0_s;

    // True uses operator centric while false uses local
    bool m_fieldCentric = true;
    // True uses axis while false uses polar
    bool m_axisLimited = true;

    // Axis Limited
    double deadbandTran = 0.0;
    // Polar Limited
    double deadbandMag = 0.0;
    // Angular Component
    double deadbandA = 0.0;

    // Axis Limited
    frc::SlewRateLimiter<units::scalar> m_limiterX;
    frc::SlewRateLimiter<units::scalar> m_limiterY;
    // Polar Limited
    frc::SlewRateLimiter<units::scalar> m_limiterMag;
    frc::Rotation2d m_currentTranslationDir{};
    units::dimensionless::scalar_t m_currentTranslationMag{0.0};
    // Angular Component (Independent)
    frc::SlewRateLimiter<units::scalar> m_limiterA;
};

#endif