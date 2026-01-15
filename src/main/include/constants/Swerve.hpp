#pragma once
#ifndef SWERVE_CONSTANTS_H
#define SWERVE_CONSTANTS_H

namespace Drive {

namespace ControllerPorts {
// USB ID for joystick for driver
constexpr int kDriverControllerPort = 0;
}

namespace TeleopOperator {
// Slew rate limiter for input controller
constexpr units::hertz_t kLimiter = units::scalar_t{1} / 1_s;
// Minimum percent of joystick distance before robot response (move)
constexpr double kDriveDeadband = 0.15;
// Minimum percent of joystick twist distance before robot response (angle)
constexpr double kDriveAngleDeadband = 0.15;
// Maximum speed that the robot will move (limited by physical design)
constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;
// Speed when speed throttle is less that precision throttle threshold
constexpr units::meters_per_second_t kDrivePrecision = 0.6_mps;
// What the throttle value will be on startup
constexpr double kDefaultThrottleXbox = 0.7;
// Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedMax = 3.0_rad_per_s;
// Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedPrecision = 0.5_rad_per_s;

constexpr units::second_t kDebounce = 0.1_s;
constexpr units::second_t kVisionDebounce = 0.35_s;
}

#endif
