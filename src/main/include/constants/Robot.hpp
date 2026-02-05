#pragma once
#ifndef CONSTANTS_ROBOT_H

#include <units/length.h>

namespace Robot {

namespace Mechanism {
// How long the base of the robot is
constexpr units::inch_t kWheelBase = 27.5_in;
// How wide the base of the robot is
constexpr units::inch_t kTrackWidth = 27.5_in;
}  // namespace Mechanism

}  // namespace Robot
#endif