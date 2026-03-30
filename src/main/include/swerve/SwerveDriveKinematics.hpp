
#ifndef SWERVE_KINEMATICS_H
#define SWERVE_KINEMATICS_H
#pragma once

#include <frc/EigenCore.h>
#include <Eigen/QR>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <wpi/array.h>

using SwerveDriveKinematics = frc::SwerveDriveKinematics<4>;

#endif