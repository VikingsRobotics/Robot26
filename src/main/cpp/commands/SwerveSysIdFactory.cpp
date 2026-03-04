#include "commands/SwerveSysIdFactory.hpp"
#include "swerve/SwerveRequest.hpp"
#include <memory>


static constexpr frc2::sysid::Direction ConvertDirectionToFRC(SwerveSysIdFactory::Direction direction) {
    return direction == SwerveSysIdFactory::Direction::kForward ? frc2::sysid::Direction::kForward : frc2::sysid::kReverse;
}

frc2::CommandPtr SwerveSysIdFactory::Translation(SwerveSubsystem* subsystem, Direction direction, Type type, TranslationConfigs configs) {
    frc2::sysid::SysIdRoutine routine{
        frc2::sysid::Config{configs.rampRate, configs.stepVoltage, configs.timeout, nullptr},
        frc2::sysid::Mechanism{[subsystem](units::volt_t volt) { subsystem->SetControl(SysIdSwerveTranslation{}.WithVolts(volt)); },
                               [subsystem](frc::sysid::SysIdRoutineLog* log) {
                                   std::span<std::unique_ptr<SwerveModule>, 4> modules = subsystem->GetModules();
                                   log->Motor("Front Left")
                                       .voltage(modules[0]->GetDriveMotor().GetMotorVoltage().GetValue())
                                       .current(modules[0]->GetDriveMotor().GetTorqueCurrent().GetValue())
                                       .position(modules[0]->GetDriveMotor().GetPosition().GetValue())
                                       .velocity(modules[0]->GetDriveMotor().GetVelocity().GetValue())
                                       .acceleration(modules[0]->GetDriveMotor().GetAcceleration().GetValue());
                                   log->Motor("Front Right")
                                       .voltage(modules[1]->GetDriveMotor().GetMotorVoltage().GetValue())
                                       .current(modules[1]->GetDriveMotor().GetTorqueCurrent().GetValue())
                                       .position(modules[1]->GetDriveMotor().GetPosition().GetValue())
                                       .velocity(modules[1]->GetDriveMotor().GetVelocity().GetValue())
                                       .acceleration(modules[1]->GetDriveMotor().GetAcceleration().GetValue());
                                   log->Motor("Back Left")
                                       .voltage(modules[2]->GetDriveMotor().GetMotorVoltage().GetValue())
                                       .current(modules[2]->GetDriveMotor().GetTorqueCurrent().GetValue())
                                       .position(modules[2]->GetDriveMotor().GetPosition().GetValue())
                                       .velocity(modules[2]->GetDriveMotor().GetVelocity().GetValue())
                                       .acceleration(modules[2]->GetDriveMotor().GetAcceleration().GetValue());
                                   log->Motor("Back Right")
                                       .voltage(modules[3]->GetDriveMotor().GetMotorVoltage().GetValue())
                                       .current(modules[3]->GetDriveMotor().GetTorqueCurrent().GetValue())
                                       .position(modules[3]->GetDriveMotor().GetPosition().GetValue())
                                       .velocity(modules[3]->GetDriveMotor().GetVelocity().GetValue())
                                       .acceleration(modules[3]->GetDriveMotor().GetAcceleration().GetValue());
                               },
                               subsystem, "Sysid-Swerve-Translation"}};

    if (type == Type::kDynamic) {
        return routine.Dynamic(ConvertDirectionToFRC(direction));
    }

    return routine.Quasistatic(ConvertDirectionToFRC(direction));
}

frc2::CommandPtr SwerveSysIdFactory::Steer(SwerveSubsystem* subsystem, Direction direction, Type type, SteerConfigs configs) {
    frc2::sysid::SysIdRoutine routine{
        frc2::sysid::Config{configs.rampRate, configs.stepVoltage, configs.timeout, nullptr},
        frc2::sysid::Mechanism{[subsystem](units::volt_t volt) { subsystem->SetControl(SysIdSwerveSteerGains{}.WithVolts(volt)); },
                               [subsystem](frc::sysid::SysIdRoutineLog* log) {
                                   std::span<std::unique_ptr<SwerveModule>, 4> modules = subsystem->GetModules();
                                   log->Motor("Front Left")
                                       .voltage(units::volt_t{modules[0]->GetAzimuthMotor().GetBusVoltage() * modules[0]->GetAzimuthMotor().GetAppliedOutput()})
                                       .current(units::ampere_t{modules[0]->GetAzimuthMotor().GetOutputCurrent()})
                                       .position(units::turn_t{modules[0]->GetAzimuthEncoder().GetPosition()})
                                       .velocity(units::turns_per_second_t{modules[0]->GetAzimuthEncoder().GetVelocity()});
                                   log->Motor("Front Right")
                                       .voltage(units::volt_t{modules[1]->GetAzimuthMotor().GetBusVoltage() * modules[1]->GetAzimuthMotor().GetAppliedOutput()})
                                       .current(units::ampere_t{modules[1]->GetAzimuthMotor().GetOutputCurrent()})
                                       .position(units::turn_t{modules[1]->GetAzimuthEncoder().GetPosition()})
                                       .velocity(units::turns_per_second_t{modules[1]->GetAzimuthEncoder().GetVelocity()});
                                   log->Motor("Back Left")
                                       .voltage(units::volt_t{modules[2]->GetAzimuthMotor().GetBusVoltage() * modules[2]->GetAzimuthMotor().GetAppliedOutput()})
                                       .current(units::ampere_t{modules[2]->GetAzimuthMotor().GetOutputCurrent()})
                                       .position(units::turn_t{modules[2]->GetAzimuthEncoder().GetPosition()})
                                       .velocity(units::turns_per_second_t{modules[2]->GetAzimuthEncoder().GetVelocity()});
                                   log->Motor("Back Right")
                                       .voltage(units::volt_t{modules[3]->GetAzimuthMotor().GetBusVoltage() * modules[3]->GetAzimuthMotor().GetAppliedOutput()})
                                       .current(units::ampere_t{modules[3]->GetAzimuthMotor().GetOutputCurrent()})
                                       .position(units::turn_t{modules[3]->GetAzimuthEncoder().GetPosition()})
                                       .velocity(units::turns_per_second_t{modules[3]->GetAzimuthEncoder().GetVelocity()});
                               },
                               subsystem, "Sysid-Swerve-Steer"}};

    if (type == Type::kDynamic) {
        return routine.Dynamic(ConvertDirectionToFRC(direction));
    }

    return routine.Quasistatic(ConvertDirectionToFRC(direction));
}

frc2::CommandPtr SwerveSysIdFactory::Rotation(SwerveSubsystem* subsystem, Direction direction, Type type, RotationConfigs configs) {
    std::shared_ptr<std::atomic<units::volt_t>> outputRate = std::make_shared<std::atomic<units::volt_t>>(0_V);

    frc2::sysid::SysIdRoutine routine{frc2::sysid::Config{configs.rampRate * 1_V / 1_tr, configs.stepTurn * 1_V / 1_tr, configs.timeout, nullptr},
                                      frc2::sysid::Mechanism{[outputRate, subsystem](units::volt_t volt) {
                                                                 outputRate->store(volt, std::memory_order::relaxed);
                                                                 subsystem->SetControl(SysIdSwerveRotation{}.WithRotationalRate(volt * 1_tr / (1_V * 1_s)));
                                                             },
                                                             [outputRate, subsystem](frc::sysid::SysIdRoutineLog* log) {
                                                                 log->Motor("System")
                                                                     .voltage(outputRate->load(std::memory_order::relaxed))
                                                                     .position(subsystem->GetPigeon2().GetYaw().GetValue())
                                                                     .velocity(subsystem->GetPigeon2().GetAngularVelocityZWorld().GetValue());
                                                                 log->Motor("Motor")
                                                                     .voltage(subsystem->GetModule(0).GetDriveMotor().GetMotorVoltage().GetValue())
                                                                     .current(subsystem->GetModule(0).GetDriveMotor().GetTorqueCurrent().GetValue())
                                                                     .position(subsystem->GetModule(0).GetDriveMotor().GetPosition().GetValue())
                                                                     .velocity(subsystem->GetModule(0).GetDriveMotor().GetVelocity().GetValue())
                                                                     .acceleration(subsystem->GetModule(0).GetDriveMotor().GetAcceleration().GetValue());
                                                             },
                                                             subsystem, "Sysid-Swerve-Rotation"}};

    if (type == Type::kDynamic) {
        return routine.Dynamic(ConvertDirectionToFRC(direction));
    }

    return routine.Quasistatic(ConvertDirectionToFRC(direction));
}