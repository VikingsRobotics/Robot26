#ifndef SWERVE_POSE_ESTIMATOR_H
#define SWERVE_POSE_ESTIMATOR_H
#pragma once

#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <map>

#include "swerve/SwerveDriveOdometry3d.hpp"


/**
 * \brief This class wraps Swerve Drive Odometry to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is
 * intended to be a drop-in for SwerveDriveOdometry.
 *
 * Update() should be called every robot loop.
 *
 * AddVisionMeasurement() can be called as infrequently as you want; if you
 * never call it, then this class will behave as regular encoder odometry.
 */
class SwerveDrivePoseEstimator3d {
public:
    using WheelSpeeds = wpi::array<frc::SwerveModuleState, 4>;
    using WheelPositions = wpi::array<frc::SwerveModulePosition, 4>;

public:
    /**
     * \brief Constructs a SwerveDrivePoseEstimator with default standard deviations
     * for the model and vision measurements.
     *
     * The default standard deviations of the model states are
     * 0.1 meters for x, 0.1 meters for y, 0.1 meters for z, and 0.1 radians for heading.
     * The default standard deviations of the vision measurements are
     * 0.9 meters for x, 0.9 meters for y, 0.9 meters for z, and 0.9 radians for heading.
     *
     * \param kinematics A correctly-configured kinematics object for your
     *     drivetrain.
     * \param gyroAngle The current gyro angle.
     * \param modulePositions The current distance and rotation measurements of
     *     the swerve modules.
     * \param initialPose The starting pose estimate.
     */
    SwerveDrivePoseEstimator3d(SwerveDriveKinematics const& kinematics, frc::Rotation3d const& gyroAngle, WheelPositions modulePositions,
                               frc::Pose3d initialPose)
        : SwerveDrivePoseEstimator3d{kinematics, gyroAngle, std::move(modulePositions), std::move(initialPose), {0.1, 0.1, 0.1, 0.1}, {0.9, 0.9, 0.9, 0.9}} {}

    /**
     * \brief Constructs a SwerveDrivePoseEstimator.
     *
     * \param kinematics A correctly-configured kinematics object for your
     *     drivetrain.
     * \param gyroAngle The current gyro angle.
     * \param modulePositions The current distance and rotation measurements of
     *     the swerve modules.
     * \param initialPose The starting pose estimate.
     * \param stateStdDevs Standard deviations of the pose estimate (x position in
     *     meters, y position in meters, and heading in radians). Increase these
     *     numbers to trust your state estimate less.
     * \param visionMeasurementStdDevs Standard deviations of the vision pose
     *     measurement (x position in meters, y position in meters, and heading in
     *     radians). Increase these numbers to trust the vision pose measurement
     *     less.
     */
    SwerveDrivePoseEstimator3d(SwerveDriveKinematics const& kinematics, frc::Rotation3d const& gyroAngle, WheelPositions modulePositions,
                               frc::Pose3d initialPose, std::array<double, 4> const& stateStdDevs, std::array<double, 4> const& visionMeasurementStdDevs);

    /**
     * \brief Sets the pose estimator's trust in robot odometry. This might be used
     * to change trust in odometry after an impact with the wall or traversing a bump.
     *
     * \param stateStdDevs Standard deviations of the pose estimate (x position in
     *     meters, y position in meters, and heading in radians). Increase these
     *     numbers to trust your state estimate less.
     */
    void SetStateStdDevs(std::array<double, 4> const& stateStdDevs);

    /**
     * \brief Sets the pose estimator's trust in vision measurements. This might be used
     * to change trust in vision measurements after the autonomous period, or to
     * change trust as distance to a vision target increases.
     *
     * \param visionMeasurementStdDevs Standard deviations of the vision pose
     *     measurement (x position in meters, y position in meters, and heading in
     *     radians). Increase these numbers to trust the vision pose measurement
     *     less.
     */
    void SetVisionMeasurementStdDevs(std::array<double, 4> const& visionMeasurementStdDevs);

    /**
     * \brief Resets the robot's position on the field.
     *
     * The gyroscope angle does not need to be reset in the user's robot code.
     * The library automatically takes care of offsetting the gyro angle.
     *
     * \param gyroAngle The current gyro angle.
     * \param wheelPositions The distances traveled by the encoders.
     * \param pose The estimated pose of the robot on the field.
     */
    void ResetPosition(frc::Rotation3d const& gyroAngle, WheelPositions wheelPositions, frc::Pose3d const& pose);

    /**
     * \brief Resets the robot's pose.
     *
     * \param pose The pose to reset to.
     */
    void ResetPose(frc::Pose3d const& pose);

    /**
     * \brief Resets the robot's translation.
     *
     * \param translation The pose to translation to.
     */
    void ResetTranslation(frc::Translation3d const& translation);

    /**
     * \brief Resets the robot's rotation.
     *
     * \param rotation The rotation to reset to.
     */
    void ResetRotation(frc::Rotation3d const& rotation);

    /**
     * \brief Gets the estimated robot pose.
     *
     * \returns The estimated robot pose in meters.
     */
    frc::Pose3d GetEstimatedPosition() const;

    /**
     * \brief Return the pose at a given timestamp, if the buffer is not empty.
     *
     * \param timestamp The pose's timestamp.
     * \returns The pose at the given timestamp (or std::nullopt if the buffer is
     * empty).
     */
    std::optional<frc::Pose3d> SampleAt(units::second_t timestamp) const;

    /**
     * \brief Adds a vision measurement to the Kalman Filter. This will correct
     * the odometry pose estimate while still accounting for measurement noise.
     *
     * This method can be called as infrequently as you want, as long as you are
     * calling Update() every loop.
     *
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * \param visionRobotPose The pose of the robot as measured by the vision
     *     camera.
     * \param timestamp The timestamp of the vision measurement in seconds. Note
     *     that if you don't use your own time source by calling UpdateWithTime(),
     *     then you must use a timestamp with an epoch since system startup (i.e.,
     *     the epoch of this timestamp is the same epoch as utils#GetCurrentTime().
     *     This means that you should use utils#GetCurrentTime() as your time source
     *     in this case.
     */
    void AddVisionMeasurement(frc::Pose3d const& visionRobotPose, units::second_t timestamp);

    /**
     * \brief Adds a vision measurement to the Kalman Filter. This will correct
     * the odometry pose estimate while still accounting for measurement noise.
     *
     * This method can be called as infrequently as you want, as long as you are
     * calling Update() every loop.
     *
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * Note that the vision measurement standard deviations passed into this
     * method will continue to apply to future measurements until a subsequent
     * call to SetVisionMeasurementStdDevs() or this method.
     *
     * \param visionRobotPose The pose of the robot as measured by the vision
     *     camera.
     * \param timestamp The timestamp of the vision measurement in seconds. Note
     *     that if you don't use your own time source by calling UpdateWithTime(),
     *     then you must use a timestamp with an epoch since system startup (i.e.,
     *     the epoch of this timestamp is the same epoch as utils#GetCurrentTime().
     *     This means that you should use utils#GetCurrentTime() as your time source
     *     in this case.
     * \param visionMeasurementStdDevs Standard deviations of the vision pose
     *     measurement (x position in meters, y position in meters, and heading in
     *     radians). Increase these numbers to trust the vision pose measurement
     *     less.
     */
    void AddVisionMeasurement(frc::Pose3d const& visionRobotPose, units::second_t timestamp, std::array<double, 4> const& visionMeasurementStdDevs);

    /**
     * \brief Updates the pose estimator with wheel encoder and gyro information.
     * This should be called every loop.
     *
     * \param gyroAngle      The current gyro angle.
     * \param wheelPositions The distances traveled by the encoders.
     *
     * \returns The estimated pose of the robot in meters.
     */
    frc::Pose3d Update(frc::Rotation3d const& gyroAngle, WheelPositions const& wheelPositions);

    /**
     * \brief Updates the pose estimator with wheel encoder and gyro information. This
     * should be called every loop.
     *
     * \param currentTime   The time at which this method was called.
     * \param gyroAngle     The current gyro angle.
     * \param wheelPositions The distances traveled by the encoders.
     *
     * \returns The estimated pose of the robot in meters.
     */
    frc::Pose3d UpdateWithTime(units::second_t currentTime, frc::Rotation3d const& gyroAngle, WheelPositions const& wheelPositions);

private:
    /**
     * \brief Removes stale vision updates that won't affect sampling.
     */
    void CleanUpVisionUpdates();

    /**
     * \brief Updates the vision matrices to account for changes in standard deviations.
     */
    void UpdateVisionMatrices();

private:
    struct VisionUpdate {
        /** \brief The vision-compensated pose estimate */
        frc::Pose3d visionPose;

        /** \brief The pose estimated based solely on odometry */
        frc::Pose3d odometryPose;

        /**
         * \brief Returns the vision-compensated version of the pose. Specifically, changes
         * the pose from being relative to this record's odometry pose to being
         * relative to this record's vision pose.
         *
         * \param pose The pose to compensate.
         * \returns The compensated pose.
         */
        frc::Pose3d Compensate(const frc::Pose3d& pose) const {
            auto const delta = pose - odometryPose;
            return visionPose + delta;
        }
    };

    static constexpr units::second_t kBufferDuration = 1.5_s;

    SwerveDriveOdometry3d m_odometry;
    std::array<double, 4> m_q{};
    std::array<double, 4> m_r{};

    frc::Matrixd<6, 6> m_visionK = frc::Matrixd<6, 6>::Zero();

    /* Maps timestamps to odometry-only pose estimates */
    frc::TimeInterpolatableBuffer<frc::Pose3d> m_odometryPoseBuffer{kBufferDuration};
    /*
     * Maps timestamps to vision updates
     * Always contains one entry before the oldest entry in m_odometryPoseBuffer,
     * unless there have been no vision measurements after the last reset
     */
    std::map<units::second_t, VisionUpdate> m_visionUpdates;

    frc::Pose3d m_poseEstimate;
};

#endif