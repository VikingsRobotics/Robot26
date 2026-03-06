#ifndef FACTORY_SWERVE_SYSID_H
#define FACTORY_SWERVE_SYSID_H
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "subsystems/SwerveSubsystem.hpp"


class SysIdFactory {
public:
    enum class Direction { kForward, kReverse };
    enum class Type { kDynamic, kQuasistatic };

    using ramp_rate_t = units::unit_t<units::compound_unit<units::volt, units::inverse<units::second>>>;

    struct TranslationConfigs {
        ramp_rate_t rampRate;
        units::volt_t stepVoltage;
        units::second_t timeout;
    };

    struct SteerConfigs {
        ramp_rate_t rampRate;
        units::volt_t stepVoltage;
        units::second_t timeout;
    };

    struct RotationConfigs {
        units::turns_per_second_t rampRate;
        units::turn_t stepTurn;
        units::second_t timeout;
    };

public:
    static frc2::CommandPtr Translation(SwerveSubsystem* subsystem, Direction direction, Type type, TranslationConfigs configs);
    static frc2::CommandPtr Steer(SwerveSubsystem* subsystem, Direction direction, Type type, SteerConfigs configs);
    static frc2::CommandPtr Rotation(SwerveSubsystem* subsystem, Direction direction, Type type, RotationConfigs configs);
};

#endif