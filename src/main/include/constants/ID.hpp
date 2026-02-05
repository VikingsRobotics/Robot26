#pragma once
#ifndef CONSTANTS_ID_H
#define CONSTANTS_ID_H

#include <ctre/phoenix6/CANBus.hpp>

namespace DeviceIdentifier {
// CTRE: CANBus Name for contructors of CRTE software classes
constexpr ctre::phoenix6::CANBus kCANBus{""};
// REV: PDH
constexpr int kPDHId = 1;

// CTRE: CANBus Pigeon2 ID
constexpr int kGyroId = 3;
// CTRE: Falcon 500 Front Left Motor ID
constexpr int kFLDriveMotorId = 6;
// REV: Neo 550 Front Left Angle Motor ID
constexpr int kFLAngleMotorId = 7;
// CTRE: Falcon 500 Front Right Motor ID
constexpr int kFRDriveMotorId = 8;
// REV: Neo 550 Front Right Angle Motor ID
constexpr int kFRAngleMotorId = 9;
// CTRE: Falcon 500 Back Left Motor ID
constexpr int kBLDriveMotorId = 4;
// REV: Neo 550 Back Left Angle Motor ID
constexpr int kBLAngleMotorId = 5;
// CTRE: Falcon 500 Back Right Motor ID
constexpr int kBRDriveMotorId = 10;
// REV: Neo 550 Back Right Angle Motor ID
constexpr int kBRAngleMotorId = 11;
}  // namespace DeviceIdentifier

#endif