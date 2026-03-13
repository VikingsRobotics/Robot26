#include "subsystems/ShooterSubsystem.hpp"
#include "Constants.hpp"

ShooterSubsystem::ShooterSubsystem() : ShooterFlywheel(Shooter::kFlywheel) {
    SetName("Shooter Subsystem");
    RegisterTelemetry([this](const ShooterFlywheel::ShooterState& state) { Telemeterize(state); });
}

void ShooterSubsystem::Telemeterize(ShooterState const& state) {
    velocity.Set(state.Velocity());
    targetVelocity.Set(state.TargetVelocity());
    appliedOutput.Set(state.AppliedOutput());
    voltage.Set(state.Voltage());
    current.Set(state.Current());
    driveTimestamp.Set(state.Timestamp.value());
    driveOdometryFrequency.Set(1.0 / state.OdometryPeriod.value());
}