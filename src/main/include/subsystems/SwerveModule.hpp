#pragma once
#ifndef SUBSYSTEM_MODULE_H
#define SUBSYSTEM_MODULE_H

#include <units/angle.h>
#include <units/velocity.h>

#include <rev/AbsoluteEncoder.h>
#include <rev/SparkMax.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "constants/SwerveModuleConfig.hpp"
#include "subsystems/SwerveModuleControl.hpp"

class SwerveModule {
public:
    SwerveModule(SwerveModuleConfigs configs);
    SwerveModule(SwerveModule& rhs) = delete;
    SwerveModule& operator=(SwerveModule& rhs) = delete;
    SwerveModule(SwerveModule&& rhs) = delete;
    SwerveModule& operator=(SwerveModule&& rhs) = delete;

    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();

    ctre::phoenix6::hardware::TalonFX& GetDriveMotor();
    rev::spark::SparkMax& GetAzimuthMotor();
    rev::spark::SparkAbsoluteEncoder& GetAzimuthEncoder();

    SwerveModuleConfigs GetConfigs();
    void ApplyConfigs(SwerveModuleConfigs configs);

    void ResetEncoder();

    void ApplyControl(SwerveModuleControl control);

    SwerveModuleControl GetControl(frc::SwerveModuleState desired, bool cosineLimited);
    SwerveModuleControl GetControl(frc::SwerveModulePosition desired);

private:
    ctre::phoenix6::hardware::TalonFX m_drivingTalonFx;
    rev::spark::SparkMax m_azimuthSparkMax;

    rev::spark::SparkAbsoluteEncoder m_azimuthAbsoluteEncoder;

    rev::spark::SparkClosedLoopController& m_sparkLoopController;

    std::function<units::angle::turn_t()> m_getTalonPosition;
    std::function<units::angular_velocity::turns_per_second_t()> m_getTalonVelocity;

    SwerveModuleConfigs m_configs;

private:
    struct Feedforward {
        units::ampere_t current;
        units::volt_t voltage;
        units::scalar_t duty;
    };

    void OpenLoop(SwerveModuleControl& control);
    void ClosedLoop(SwerveModuleControl& control, Feedforward ff);
    void ClosedPositionLoop(SwerveModuleControl& control, Feedforward ff);
    void ClosedVelocityLoop(SwerveModuleControl& control, Feedforward ff);
    void ClosedMagicPositionLoop(SwerveModuleControl& control, Feedforward ff);
    void ClosedMagicVelocityLoop(SwerveModuleControl& control, Feedforward ff, units::meters_per_second_squared_t accel);

    Feedforward CalculateFeedforward(SwerveModuleControl& control);

    Feedforward CalculateFeedforward(units::newton_t x, units::newton_t y);
    Feedforward CalculateFeedforward(units::newton_t linear);

    Feedforward CalculateFeedforwardDuty(units::meters_per_second_squared_t accel);
    Feedforward CalculateFeedforwardVoltage(units::meters_per_second_squared_t accel);
    Feedforward CalculateFeedforwardCurrent(units::meters_per_second_squared_t accel);
};

#endif
