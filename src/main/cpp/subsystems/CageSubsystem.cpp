#include "subsystems/CageSubsystem.hpp"

#include "Constants.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

CageSubsystem::CageSubsystem() : m_solenoid{frc::PneumaticsModuleType::REVPH, Cage::SolenoidId::kForwardChannelId, Cage::SolenoidId::kReverseChannelId} {
    ExpandCage();

    SetName("Cage Subsystem");
    frc::SmartDashboard::PutData(this);
}

void CageSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Cage State", IsCageExpanded() ? "Expanded" : IsCageCompressed() ? "Compressed" : "Unmoved");
}

void CageSubsystem::ExpandCage() {
    m_solenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void CageSubsystem::CompressCage() {
    m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

bool CageSubsystem::IsCageExpanded() {
    return m_solenoid.Get() == frc::DoubleSolenoid::Value::kForward;
}

bool CageSubsystem::IsCageCompressed() {
    return m_solenoid.Get() == frc::DoubleSolenoid::Value::kReverse;
}