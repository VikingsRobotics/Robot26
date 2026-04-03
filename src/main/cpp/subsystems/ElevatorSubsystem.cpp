#include "subsystems/ElevatorSubsystem.hpp"

#include "Constants.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

ElevatorSubsystem::ElevatorSubsystem()
    : m_cylinder{frc::PneumaticsModuleType::REVPH, Elevator::SolenoidId::kForwardChannelId, Elevator::SolenoidId::kReverseChannelId} {
    PutElevatorDown();

    SetName("Elevator Subsystem");
    frc::SmartDashboard::PutData(this);
}

void ElevatorSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Elevator State", IsElevatorDown() ? "Down" : IsElevatorUp() ? "Up" : "Unmoved");
}

void ElevatorSubsystem::PutElevatorDown() {
    m_cylinder.Set(frc::DoubleSolenoid::Value::kForward);
}

void ElevatorSubsystem::PutElevatorUp() {
    m_cylinder.Set(frc::DoubleSolenoid::Value::kReverse);
}

bool ElevatorSubsystem::IsElevatorDown() {
    return m_cylinder.Get() == frc::DoubleSolenoid::Value::kForward;
}

bool ElevatorSubsystem::IsElevatorUp() {
    return m_cylinder.Get() == frc::DoubleSolenoid::Value::kReverse;
}