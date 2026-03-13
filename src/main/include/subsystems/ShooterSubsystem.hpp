#ifndef SUBSYSTEM_SHOOTER_H
#define SUBSYSTEM_SHOOTER_H
#pragma once

#include <vector>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/util/Color8Bit.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>

#include "shooter/ShooterFlywheel.hpp"

class ShooterSubsystem : public ShooterFlywheel {
public:
    ShooterSubsystem();
    ShooterSubsystem(ShooterSubsystem& rhs) = delete;
    ShooterSubsystem& operator=(ShooterSubsystem& rhs) = delete;
    ShooterSubsystem(ShooterSubsystem&& rhs) = delete;
    ShooterSubsystem& operator=(ShooterSubsystem&& rhs) = delete;

private:
    void Telemeterize(ShooterState const& state);

private:
    // What to publish over networktables for telemetry
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    // Robot flywheel state
    std::shared_ptr<nt::NetworkTable> flywheelStateTable = inst.GetTable("FlywheelState");
    nt::DoublePublisher velocity = flywheelStateTable->GetDoubleTopic("Velocity").Publish();
    nt::DoublePublisher targetVelocity = flywheelStateTable->GetDoubleTopic("TargetVelocity").Publish();
    nt::DoublePublisher appliedOutput = flywheelStateTable->GetDoubleTopic("AppliedOutput").Publish();
    nt::DoublePublisher voltage = flywheelStateTable->GetDoubleTopic("Voltage").Publish();
    nt::DoublePublisher current = flywheelStateTable->GetDoubleTopic("Current").Publish();
    nt::DoublePublisher driveTimestamp = flywheelStateTable->GetDoubleTopic("Timestamp").Publish();
    nt::DoublePublisher driveOdometryFrequency = flywheelStateTable->GetDoubleTopic("OdometryFrequency").Publish();
};

#endif
