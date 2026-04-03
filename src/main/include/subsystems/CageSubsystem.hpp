#ifndef SUBSYSTEM_CAGE_H
#define SUBSYSTEM_CAGE_H
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>

class CageSubsystem : public frc2::SubsystemBase {
public:
    CageSubsystem();

    void Periodic() override;

    void CompressCage();
    void ExpandCage();

    bool IsCageCompressed();
    bool IsCageExpanded();

private:
    frc::DoubleSolenoid m_solenoid;
};

#endif