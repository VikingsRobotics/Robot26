#ifndef SWERVE_ODOMETRY_H
#define SWERVE_ODOMETRY_H
#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>

#include "swerve/SwerveDriveKinematics.hpp"

/**
 * \brief Class for swerve drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * swerve drive encoders and swerve azimuth encoders.
 *
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
class SwerveDriveOdometry3d {
public:
    using WheelSpeeds = wpi::array<frc::SwerveModuleState, 4>;
    using WheelPositions = wpi::array<frc::SwerveModulePosition, 4>;

public:
    /**
     * \brief Constructs a SwerveDriveOdometry object.
     *
     * \param kinematics The swerve drive kinematics for your drivetrain.
     * \param gyroAngle The angle reported by the gyroscope.
     * \param modulePositions The wheel positions reported by each module.
     * \param initialPose The starting position of the robot on the field.
     */
    SwerveDriveOdometry3d(SwerveDriveKinematics const& kinematics, frc::Rotation3d const& gyroAngle, WheelPositions modulePositions,
                          frc::Pose3d initialPose = frc::Pose3d{});

    /**
     * \brief Resets the robot's position on the field.
     *
     * The gyroscope angle does not need to be reset here on the user's robot
     * code. The library automatically takes care of offsetting the gyro angle.
     *
     * \param gyroAngle The angle reported by the gyroscope.
     * \param wheelPositions The current distances measured by each wheel.
     * \param pose The position on the field that your robot is at.
     */
    void ResetPosition(frc::Rotation3d const& gyroAngle, WheelPositions wheelPositions, frc::Pose3d const& pose);

    /**
     * \brief Resets the pose.
     *
     * \param pose The pose to reset to.
     */
    void ResetPose(frc::Pose3d const& pose);

    /**
     * \brief Resets the translation of the pose.
     *
     * \param translation The translation to reset to.
     */
    void ResetTranslation(frc::Translation3d const& translation);

    /**
     * \brief Resets the rotation of the pose.
     *
     * \param rotation The rotation to reset to.
     */
    void ResetRotation(frc::Rotation3d const& rotation);

    /**
     * \brief Returns the position of the robot on the field.
     *
     * \returns The pose of the robot.
     */
    frc::Pose3d const& Pose() const;

    /**
     * \brief Updates the robot's position on the field using forward kinematics and
     * integration of the pose over time. This method takes in an angle parameter
     * which is used instead of the angular rate that is calculated from forward
     * kinematics, in addition to the current distance measurement at each wheel.
     *
     * \param gyroAngle The angle reported by the gyroscope.
     * \param wheelPositions The current distances measured by each wheel.
     *
     * \returns The new pose of the robot.
     */
    frc::Pose3d const& Update(frc::Rotation3d const& gyroAngle, WheelPositions wheelPositions);

private:
    SwerveDriveKinematics const& m_kinematics;
    frc::Pose3d m_pose;

    WheelPositions m_previousWheelPositions;
    frc::Rotation3d m_previousAngle;
    frc::Rotation3d m_gyroOffset;
};

#endif