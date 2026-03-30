#ifndef SWERVE_ODOMETRY_H
#define SWERVE_ODOMETRY_H
#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/kinematics/SwerveDriveOdometry3d.h>

#include "swerve/SwerveDriveKinematics.hpp"

using SwerveDriveOdometry3d = frc::SwerveDriveOdometry3d<4>;

#endif