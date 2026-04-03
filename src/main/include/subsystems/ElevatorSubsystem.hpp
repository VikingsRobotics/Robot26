#ifndef SUBSYSTEM_ELEVATOR_H
#define SUBSYSTEM_ELEVATOR_H
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
public:
    ElevatorSubsystem();

    void Periodic() override;

    void PutElevatorDown();
    void PutElevatorUp();

    bool IsElevatorDown();
    bool IsElevatorUp();

private:
    frc::DoubleSolenoid m_cylinder;
};

#endif