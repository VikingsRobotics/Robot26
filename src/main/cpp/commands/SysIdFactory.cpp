#include "commands/SysIdFactory.hpp"
#include "swerve/SwerveRequest.hpp"
#include <memory>

static constexpr frc2::sysid::Direction ConvertDirectionToFRC(SysIdFactory::Direction direction) {
    return direction == SysIdFactory::Direction::kForward ? frc2::sysid::Direction::kForward : frc2::sysid::kReverse;
}

frc2::CommandPtr SysIdFactory::Translation(SwerveSubsystem* subsystem, Direction direction, Type type, TranslationConfigs configs) {
    frc2::sysid::SysIdRoutine routine{
        frc2::sysid::Config{configs.rampRate, configs.stepVoltage, configs.timeout, nullptr},
        frc2::sysid::Mechanism{[subsystem](units::volt_t volt) { subsystem->SetControl(SysIdSwerveTranslation{}.WithVolts(volt)); },
                               [subsystem](frc::sysid::SysIdRoutineLog* log) {
                                   std::array<SwerveModule::CacheState, 4> states = subsystem->GetModulesCachedStates();
                                   log->Motor("Front Left")
                                       .voltage(states[0].drive.voltage)
                                       .current(states[0].drive.current)
                                       .position(states[0].drive.position)
                                       .velocity(states[0].drive.velocity)
                                       .acceleration(states[0].drive.acceleration);
                                   log->Motor("Front Right")
                                       .voltage(states[1].drive.voltage)
                                       .current(states[1].drive.current)
                                       .position(states[1].drive.position)
                                       .velocity(states[1].drive.velocity)
                                       .acceleration(states[1].drive.acceleration);
                                   log->Motor("Back Left")
                                       .voltage(states[2].drive.voltage)
                                       .current(states[2].drive.current)
                                       .position(states[2].drive.position)
                                       .velocity(states[2].drive.velocity)
                                       .acceleration(states[2].drive.acceleration);
                                   log->Motor("Back Right")
                                       .voltage(states[3].drive.voltage)
                                       .current(states[3].drive.current)
                                       .position(states[3].drive.position)
                                       .velocity(states[3].drive.velocity)
                                       .acceleration(states[3].drive.acceleration);
                               },
                               subsystem, "Sysid-Swerve-Translation"}};

    if (type == Type::kDynamic) {
        return routine.Dynamic(ConvertDirectionToFRC(direction));
    }

    return routine.Quasistatic(ConvertDirectionToFRC(direction));
}

frc2::CommandPtr SysIdFactory::Steer(SwerveSubsystem* subsystem, Direction direction, Type type, SteerConfigs configs) {
    frc2::sysid::SysIdRoutine routine{
        frc2::sysid::Config{configs.rampRate, configs.stepVoltage, configs.timeout, nullptr},
        frc2::sysid::Mechanism{[subsystem](units::volt_t volt) { subsystem->SetControl(SysIdSwerveSteerGains{}.WithVolts(volt)); },
                               [subsystem](frc::sysid::SysIdRoutineLog* log) {
                                   std::array<SwerveModule::CacheState, 4> states = subsystem->GetModulesCachedStates();
                                   log->Motor("Front Left")
                                       .voltage(states[0].azimuth.voltage)
                                       .current(states[0].azimuth.current)
                                       .position(states[0].azimuth.position)
                                       .velocity(states[0].azimuth.velocity);
                                   log->Motor("Front Right")
                                       .voltage(states[1].azimuth.voltage)
                                       .current(states[1].azimuth.current)
                                       .position(states[1].azimuth.position)
                                       .velocity(states[1].azimuth.velocity);
                                   log->Motor("Back Left")
                                       .voltage(states[2].azimuth.voltage)
                                       .current(states[2].azimuth.current)
                                       .position(states[2].azimuth.position)
                                       .velocity(states[2].azimuth.velocity);
                                   log->Motor("Back Right")
                                       .voltage(states[3].azimuth.voltage)
                                       .current(states[3].azimuth.current)
                                       .position(states[3].azimuth.position)
                                       .velocity(states[3].azimuth.velocity);
                               },
                               subsystem, "Sysid-Swerve-Steer"}};

    if (type == Type::kDynamic) {
        return routine.Dynamic(ConvertDirectionToFRC(direction));
    }

    return routine.Quasistatic(ConvertDirectionToFRC(direction));
}

frc2::CommandPtr SysIdFactory::Rotation(SwerveSubsystem* subsystem, Direction direction, Type type, RotationConfigs configs) {
    std::shared_ptr<std::atomic<units::volt_t>> outputRate = std::make_shared<std::atomic<units::volt_t>>(0_V);

    frc2::sysid::SysIdRoutine routine{frc2::sysid::Config{configs.rampRate * 1_V / 1_tr, configs.stepTurn * 1_V / 1_tr, configs.timeout, nullptr},
                                      frc2::sysid::Mechanism{[outputRate, subsystem](units::volt_t volt) {
                                                                 outputRate->store(volt, std::memory_order::relaxed);
                                                                 subsystem->SetControl(SysIdSwerveRotation{}.WithRotationalRate(volt * 1_tr / (1_V * 1_s)));
                                                             },
                                                             [outputRate, subsystem](frc::sysid::SysIdRoutineLog* log) {
                                                                 SwerveModule::CacheState state = subsystem->GetModule(0).GetCachedState();

                                                                 log->Motor("System")
                                                                     .voltage(outputRate->load(std::memory_order::relaxed))
                                                                     .position(subsystem->GetPigeon2().GetYaw().GetValue())
                                                                     .velocity(subsystem->GetPigeon2().GetAngularVelocityZWorld().GetValue());
                                                                 log->Motor("Motor")
                                                                     .voltage(state.drive.voltage)
                                                                     .current(state.drive.current)
                                                                     .position(state.drive.position)
                                                                     .velocity(state.drive.velocity)
                                                                     .acceleration(state.drive.acceleration);
                                                             },
                                                             subsystem, "Sysid-Swerve-Rotation"}};

    if (type == Type::kDynamic) {
        return routine.Dynamic(ConvertDirectionToFRC(direction));
    }

    return routine.Quasistatic(ConvertDirectionToFRC(direction));
}
