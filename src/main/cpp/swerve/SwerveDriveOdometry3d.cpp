#include "swerve/SwerveDriveOdometry3d.hpp"

SwerveDriveOdometry3d::SwerveDriveOdometry3d(SwerveDriveKinematics const& kinematics, frc::Rotation3d const& gyroAngle, WheelPositions modulePositions,
                                             frc::Pose3d initialPose)
    : m_kinematics{kinematics},
      m_pose{std::move(initialPose)},
      m_previousWheelPositions{std::move(modulePositions)},
      m_previousAngle{m_pose.Rotation()},
      m_gyroOffset{m_pose.Rotation() - gyroAngle} {}

void SwerveDriveOdometry3d::ResetPosition(frc::Rotation3d const& gyroAngle, WheelPositions wheelPositions, frc::Pose3d const& pose) {
    m_pose = pose;
    m_previousAngle = m_pose.Rotation();
    m_gyroOffset = m_pose.Rotation() - gyroAngle;
    m_previousWheelPositions = std::move(wheelPositions);
}

void SwerveDriveOdometry3d::ResetPose(frc::Pose3d const& pose) {
    m_gyroOffset = m_gyroOffset + (pose.Rotation() - m_pose.Rotation());
    m_pose = pose;
    m_previousAngle = pose.Rotation();
}

void SwerveDriveOdometry3d::ResetTranslation(frc::Translation3d const& translation) {
    m_pose = frc::Pose3d{translation, m_pose.Rotation()};
}

void SwerveDriveOdometry3d::ResetRotation(frc::Rotation3d const& rotation) {
    m_gyroOffset = m_gyroOffset + (rotation - m_pose.Rotation());
    m_pose = frc::Pose3d{m_pose.Translation(), rotation};
    m_previousAngle = rotation;
}

frc::Pose3d const& SwerveDriveOdometry3d::Pose() const {
    return m_pose;
}

frc::Pose3d const& SwerveDriveOdometry3d::Update(frc::Rotation3d const& gyroAngle, WheelPositions wheelPositions) {
    frc::Rotation3d angle = gyroAngle + m_gyroOffset;
    Eigen::Vector3d angle_difference = (angle - m_previousAngle).ToVector();

    auto twist2d = m_kinematics.ToTwist2d(m_previousWheelPositions, wheelPositions);
    frc::Twist3d twist{
        twist2d.dx, twist2d.dy, 0_m, units::radian_t{angle_difference(0)}, units::radian_t{angle_difference(1)}, units::radian_t{angle_difference(2)}};

    frc::Pose3d newPose = m_pose.Exp(twist);

    m_previousWheelPositions = wheelPositions;
    m_previousAngle = angle;
    m_pose = {newPose.Translation(), angle};

    return m_pose;
}