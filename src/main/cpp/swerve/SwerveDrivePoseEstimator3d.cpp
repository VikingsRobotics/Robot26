#include "swerve/SwerveDrivePoseEstimator3d.hpp"

#ifdef SWERVE_POSE_ESTIMATOR_CUSTOM

#include "wpimath/MathShared.h"

#include <frc/Timer.h>


SwerveDrivePoseEstimator3d::SwerveDrivePoseEstimator3d(SwerveDriveKinematics const& kinematics, frc::Rotation3d const& gyroAngle,
                                                       WheelPositions modulePositions, frc::Pose3d initialPose, std::array<double, 4> const& stateStdDevs,
                                                       std::array<double, 4> const& visionMeasurementStdDevs)
    : m_odometry{kinematics, gyroAngle, modulePositions, initialPose} {
    wpi::math::MathSharedStore::ReportUsage(wpi::math::MathUsageId::kEstimator_PoseEstimator3d, 1);

    m_q[0] = stateStdDevs[0] * stateStdDevs[0];
    m_q[1] = stateStdDevs[1] * stateStdDevs[1];
    m_q[2] = stateStdDevs[2] * stateStdDevs[2];
    m_q[3] = stateStdDevs[3] * stateStdDevs[3];
    m_r[0] = visionMeasurementStdDevs[0] * visionMeasurementStdDevs[0];
    m_r[1] = visionMeasurementStdDevs[1] * visionMeasurementStdDevs[1];
    m_r[2] = visionMeasurementStdDevs[2] * visionMeasurementStdDevs[2];
    m_r[3] = visionMeasurementStdDevs[3] * visionMeasurementStdDevs[3];
    UpdateVisionMatrices();
}

void SwerveDrivePoseEstimator3d::SetStateStdDevs(std::array<double, 4> const& stateStdDevs) {
    m_q[0] = stateStdDevs[0] * stateStdDevs[0];
    m_q[1] = stateStdDevs[1] * stateStdDevs[1];
    m_q[2] = stateStdDevs[2] * stateStdDevs[2];
    m_q[3] = stateStdDevs[3] * stateStdDevs[3];
    UpdateVisionMatrices();
}

void SwerveDrivePoseEstimator3d::SetVisionMeasurementStdDevs(std::array<double, 4> const& visionMeasurementStdDevs) {
    m_r[0] = visionMeasurementStdDevs[0] * visionMeasurementStdDevs[0];
    m_r[1] = visionMeasurementStdDevs[1] * visionMeasurementStdDevs[1];
    m_r[2] = visionMeasurementStdDevs[2] * visionMeasurementStdDevs[2];
    m_r[3] = visionMeasurementStdDevs[3] * visionMeasurementStdDevs[3];
    UpdateVisionMatrices();
}

void SwerveDrivePoseEstimator3d::ResetPosition(frc::Rotation3d const& gyroAngle, WheelPositions wheelPositions, frc::Pose3d const& pose) {
    m_odometry.ResetPosition(gyroAngle, std::move(wheelPositions), pose);
    m_odometryPoseBuffer.Clear();
    m_visionUpdates.clear();
    m_poseEstimate = m_odometry.GetPose();
}

void SwerveDrivePoseEstimator3d::ResetPose(frc::Pose3d const& pose) {
    m_odometry.ResetPose(pose);
    m_odometryPoseBuffer.Clear();
    m_visionUpdates.clear();
    m_poseEstimate = m_odometry.GetPose();
}

void SwerveDrivePoseEstimator3d::ResetTranslation(frc::Translation3d const& translation) {
    m_odometry.ResetTranslation(translation);

    std::optional<std::pair<units::second_t, VisionUpdate>> latestVisionUpdate =
        m_visionUpdates.empty() ? std::nullopt : std::optional{*m_visionUpdates.crbegin()};
    m_odometryPoseBuffer.Clear();
    m_visionUpdates.clear();

    if (latestVisionUpdate) {
        const VisionUpdate visionUpdate{frc::Pose3d{translation, latestVisionUpdate->second.visionPose.Rotation()},
                                        frc::Pose3d{translation, latestVisionUpdate->second.odometryPose.Rotation()}};
        m_visionUpdates[latestVisionUpdate->first] = visionUpdate;
        m_poseEstimate = visionUpdate.Compensate(m_odometry.GetPose());
    } else {
        m_poseEstimate = m_odometry.GetPose();
    }
}

void SwerveDrivePoseEstimator3d::ResetRotation(frc::Rotation3d const& rotation) {
    m_odometry.ResetRotation(rotation);

    std::optional<std::pair<units::second_t, VisionUpdate>> latestVisionUpdate =
        m_visionUpdates.empty() ? std::nullopt : std::optional{*m_visionUpdates.crbegin()};
    m_odometryPoseBuffer.Clear();
    m_visionUpdates.clear();

    if (latestVisionUpdate) {
        VisionUpdate const visionUpdate{frc::Pose3d{latestVisionUpdate->second.visionPose.Translation(), rotation},
                                        frc::Pose3d{latestVisionUpdate->second.odometryPose.Translation(), rotation}};
        m_visionUpdates[latestVisionUpdate->first] = visionUpdate;
        m_poseEstimate = visionUpdate.Compensate(m_odometry.GetPose());
    } else {
        m_poseEstimate = m_odometry.GetPose();
    }
}

frc::Pose3d SwerveDrivePoseEstimator3d::GetEstimatedPosition() const {
    return m_poseEstimate;
}

std::optional<frc::Pose3d> SwerveDrivePoseEstimator3d::SampleAt(units::second_t timestamp) const {
    if (m_odometryPoseBuffer.GetInternalBuffer().empty()) {
        return std::nullopt;
    }

    units::second_t oldestOdometryTimestamp = m_odometryPoseBuffer.GetInternalBuffer().front().first;
    units::second_t newestOdometryTimestamp = m_odometryPoseBuffer.GetInternalBuffer().back().first;
    timestamp = std::clamp(timestamp, oldestOdometryTimestamp, newestOdometryTimestamp);

    if (m_visionUpdates.empty() || timestamp < m_visionUpdates.begin()->first) {
        return m_odometryPoseBuffer.Sample(timestamp);
    }

    auto floorIter = m_visionUpdates.upper_bound(timestamp);
    --floorIter;
    auto visionUpdate = floorIter->second;

    auto odometryEstimate = m_odometryPoseBuffer.Sample(timestamp);

    if (odometryEstimate) {
        return visionUpdate.Compensate(*odometryEstimate);
    }
    return std::nullopt;
}

void SwerveDrivePoseEstimator3d::AddVisionMeasurement(frc::Pose3d const& visionRobotPose, units::second_t timestamp) {
    if (m_odometryPoseBuffer.GetInternalBuffer().empty() || m_odometryPoseBuffer.GetInternalBuffer().front().first - kBufferDuration > timestamp) {
        return;
    }

    CleanUpVisionUpdates();

    auto odometrySample = m_odometryPoseBuffer.Sample(timestamp);

    if (!odometrySample) {
        return;
    }

    auto visionSample = SampleAt(timestamp);

    if (!visionSample) {
        return;
    }

    auto transform = visionRobotPose - visionSample.value();

    frc::Vectord<6> k_times_transform =
        m_visionK *
        frc::Vectord<6>{
            transform.X().value(),           transform.Y().value(), transform.Z().value(), transform.Rotation().X().value(), transform.Rotation().Y().value(),
            transform.Rotation().Z().value()};

    frc::Transform3d scaledTransform{
        units::meter_t{k_times_transform(0)}, units::meter_t{k_times_transform(1)}, units::meter_t{k_times_transform(2)},
        frc::Rotation3d{units::radian_t{k_times_transform(3)}, units::radian_t{k_times_transform(4)}, units::radian_t{k_times_transform(5)}}};

    VisionUpdate visionUpdate{*visionSample + scaledTransform, *odometrySample};
    m_visionUpdates[timestamp] = visionUpdate;

    auto firstAfter = m_visionUpdates.upper_bound(timestamp);
    m_visionUpdates.erase(firstAfter, m_visionUpdates.end());

    m_poseEstimate = visionUpdate.Compensate(m_odometry.GetPose());
}

void SwerveDrivePoseEstimator3d::AddVisionMeasurement(frc::Pose3d const& visionRobotPose, units::second_t timestamp,
                                                      std::array<double, 4> const& visionMeasurementStdDevs) {
    SetVisionMeasurementStdDevs(visionMeasurementStdDevs);
    AddVisionMeasurement(visionRobotPose, timestamp);
}

frc::Pose3d SwerveDrivePoseEstimator3d::Update(frc::Rotation3d const& gyroAngle, WheelPositions const& wheelPositions) {
    return UpdateWithTime(frc::Timer::GetTimestamp(), gyroAngle, wheelPositions);
}

frc::Pose3d SwerveDrivePoseEstimator3d::UpdateWithTime(units::second_t currentTime, frc::Rotation3d const& gyroAngle, WheelPositions const& wheelPositions) {
    auto odometryEstimate = m_odometry.Update(gyroAngle, wheelPositions);

    m_odometryPoseBuffer.AddSample(currentTime, odometryEstimate);

    if (m_visionUpdates.empty()) {
        m_poseEstimate = std::move(odometryEstimate);
    } else {
        auto const visionUpdate = m_visionUpdates.rbegin()->second;
        m_poseEstimate = visionUpdate.Compensate(odometryEstimate);
    }

    return GetEstimatedPosition();
}

void SwerveDrivePoseEstimator3d::CleanUpVisionUpdates() {
    if (m_odometryPoseBuffer.GetInternalBuffer().empty()) {
        return;
    }

    units::second_t oldestOdometryTimestamp = m_odometryPoseBuffer.GetInternalBuffer().front().first;

    if (m_visionUpdates.empty() || oldestOdometryTimestamp < m_visionUpdates.begin()->first) {
        return;
    }

    auto newestNeededVisionUpdate = m_visionUpdates.upper_bound(oldestOdometryTimestamp);
    --newestNeededVisionUpdate;

    m_visionUpdates.erase(m_visionUpdates.begin(), newestNeededVisionUpdate);
}

void SwerveDrivePoseEstimator3d::UpdateVisionMatrices() {
    for (size_t row = 0; row < 4; ++row) {
        if (m_q[row] == 0.0) {
            m_visionK(row, row) = 0.0;
        } else {
            m_visionK(row, row) = m_q[row] / (m_q[row] + std::sqrt(m_q[row] * m_r[row]));
        }
    }
    double angle_gain = m_visionK(3, 3);
    m_visionK(4, 4) = angle_gain;
    m_visionK(5, 5) = angle_gain;
}

#endif