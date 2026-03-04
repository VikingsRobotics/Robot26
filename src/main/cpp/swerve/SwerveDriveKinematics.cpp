#include "swerve/SwerveDriveKinematics.hpp"

SwerveDriveKinematics::SwerveDriveKinematics(wpi::array<frc::Translation2d, 4> moduleLocations)
    : m_moduleLocations{moduleLocations}, m_lastModuleHeading{wpi::empty_array} {
    // clang-format off
      m_inverseKinematics.template block<8, 3>(0, 0) <<
        1, 0, (-m_moduleLocations[0].Y()).value(),
        0, 1, (+m_moduleLocations[0].X()).value(),
        1, 0, (-m_moduleLocations[1].Y()).value(),
        0, 1, (+m_moduleLocations[1].X()).value(),
        1, 0, (-m_moduleLocations[2].Y()).value(),
        0, 1, (+m_moduleLocations[2].X()).value(),
        1, 0, (-m_moduleLocations[3].Y()).value(),
        0, 1, (+m_moduleLocations[3].X()).value();
    // clang-format on

    m_forwardKinematics = m_inverseKinematics.householderQr();
}

void SwerveDriveKinematics::ResetHeadings(wpi::array<frc::Rotation2d, 4> moduleHeadings) {
    m_lastModuleHeading[0] = moduleHeadings[0];
    m_lastModuleHeading[1] = moduleHeadings[1];
    m_lastModuleHeading[2] = moduleHeadings[2];
    m_lastModuleHeading[3] = moduleHeadings[3];
}

SwerveDriveKinematics::WheelSpeeds SwerveDriveKinematics::ToSwerveModuleStates(frc::ChassisSpeeds const& chassisSpeeds,
                                                                               frc::Translation2d const& centerOfRotation) {
    WheelSpeeds moduleStates(wpi::empty_array);

    if (chassisSpeeds.vx == 0_mps && chassisSpeeds.vy == 0_mps && chassisSpeeds.omega == 0_rad_per_s) {
        moduleStates[0] = {0_mps, m_lastModuleHeading[0]};
        moduleStates[1] = {0_mps, m_lastModuleHeading[1]};
        moduleStates[2] = {0_mps, m_lastModuleHeading[2]};
        moduleStates[3] = {0_mps, m_lastModuleHeading[3]};

        return moduleStates;
    }

    if (centerOfRotation != m_lastCOR) {
        // clang-format off
        m_inverseKinematics.template block<8, 3>(0, 0) <<
            1, 0, (-m_moduleLocations[0].Y() + centerOfRotation.Y()).value(),
            0, 1, (+m_moduleLocations[0].X() - centerOfRotation.X()).value(),
            1, 0, (-m_moduleLocations[1].Y() + centerOfRotation.Y()).value(),
            0, 1, (+m_moduleLocations[1].X() - centerOfRotation.X()).value(),
            1, 0, (-m_moduleLocations[2].Y() + centerOfRotation.Y()).value(),
            0, 1, (+m_moduleLocations[2].X() - centerOfRotation.X()).value(),
            1, 0, (-m_moduleLocations[3].Y() + centerOfRotation.Y()).value(),
            0, 1, (+m_moduleLocations[3].X() - centerOfRotation.X()).value();
        // clang-format on
        m_lastCOR = centerOfRotation;
    }


    Eigen::Vector3d chassisSpeedsVector{chassisSpeeds.vx.value(), chassisSpeeds.vy.value(), chassisSpeeds.omega.value()};

    frc::Matrixd<8, 1> moduleStateMatrix = m_inverseKinematics * chassisSpeedsVector;

    units::meters_per_second_t speed0 =
        units::math::hypot(units::meters_per_second_t{moduleStateMatrix(0, 0)}, units::meters_per_second_t{moduleStateMatrix(1, 0)});
    units::meters_per_second_t speed1 =
        units::math::hypot(units::meters_per_second_t{moduleStateMatrix(2, 0)}, units::meters_per_second_t{moduleStateMatrix(3, 0)});
    units::meters_per_second_t speed2 =
        units::math::hypot(units::meters_per_second_t{moduleStateMatrix(4, 0)}, units::meters_per_second_t{moduleStateMatrix(5, 0)});
    units::meters_per_second_t speed3 =
        units::math::hypot(units::meters_per_second_t{moduleStateMatrix(6, 0)}, units::meters_per_second_t{moduleStateMatrix(7, 0)});

    frc::Rotation2d rot0 = speed0 > 1e-6_mps ? frc::Rotation2d{moduleStateMatrix(0, 0), moduleStateMatrix(1, 0)} : m_lastModuleHeading[0];
    frc::Rotation2d rot1 = speed1 > 1e-6_mps ? frc::Rotation2d{moduleStateMatrix(2, 0), moduleStateMatrix(3, 0)} : m_lastModuleHeading[1];
    frc::Rotation2d rot2 = speed2 > 1e-6_mps ? frc::Rotation2d{moduleStateMatrix(4, 0), moduleStateMatrix(5, 0)} : m_lastModuleHeading[2];
    frc::Rotation2d rot3 = speed3 > 1e-6_mps ? frc::Rotation2d{moduleStateMatrix(6, 0), moduleStateMatrix(7, 0)} : m_lastModuleHeading[3];

    moduleStates[0] = {speed0, rot0};
    moduleStates[1] = {speed1, rot1};
    moduleStates[2] = {speed2, rot2};
    moduleStates[3] = {speed3, rot3};

    m_lastModuleHeading[0] = rot0;
    m_lastModuleHeading[1] = rot1;
    m_lastModuleHeading[2] = rot2;
    m_lastModuleHeading[3] = rot3;

    return moduleStates;
}

frc::ChassisSpeeds SwerveDriveKinematics::ToChassisSpeeds(WheelSpeeds const& moduleStates) const {
    frc::Matrixd<8, 1> moduleStateMatrix;
    // clang-format off
    moduleStateMatrix.template block<8,1>(0,0) <<
        moduleStates[0].speed.value() * moduleStates[0].angle.Cos(), moduleStates[0].speed.value() * moduleStates[0].angle.Sin(),
        moduleStates[1].speed.value() * moduleStates[1].angle.Cos(), moduleStates[1].speed.value() * moduleStates[1].angle.Sin(),
        moduleStates[2].speed.value() * moduleStates[2].angle.Cos(), moduleStates[2].speed.value() * moduleStates[2].angle.Sin(),
        moduleStates[3].speed.value() * moduleStates[3].angle.Cos(), moduleStates[3].speed.value() * moduleStates[3].angle.Sin();
    // clang-format on

    Eigen::Vector3d chassisSpeedsVector = m_forwardKinematics.solve(moduleStateMatrix);

    return {units::meters_per_second_t{chassisSpeedsVector(0)}, units::meters_per_second_t{chassisSpeedsVector(1)},
            units::radians_per_second_t{chassisSpeedsVector(2)}};
}

frc::Twist2d SwerveDriveKinematics::ToTwist2d(WheelPositions const& moduleDeltas) const {
    frc::Matrixd<8, 1> moduleDeltaMatrix;

    // clang-format off
    moduleDeltaMatrix.template block<8,1>(0,0) <<
        moduleDeltas[0].distance.value() * moduleDeltas[0].angle.Cos(), moduleDeltas[0].distance.value() * moduleDeltas[0].angle.Sin(),
        moduleDeltas[1].distance.value() * moduleDeltas[1].angle.Cos(), moduleDeltas[1].distance.value() * moduleDeltas[1].angle.Sin(),
        moduleDeltas[2].distance.value() * moduleDeltas[2].angle.Cos(), moduleDeltas[2].distance.value() * moduleDeltas[2].angle.Sin(),
        moduleDeltas[3].distance.value() * moduleDeltas[3].angle.Cos(), moduleDeltas[3].distance.value() * moduleDeltas[3].angle.Sin();
    // clang-format on

    Eigen::Vector3d chassisDeltaVector = m_forwardKinematics.solve(moduleDeltaMatrix);

    return {units::meter_t{chassisDeltaVector(0)}, units::meter_t{chassisDeltaVector(1)}, units::radian_t{chassisDeltaVector(2)}};
}

frc::Twist2d SwerveDriveKinematics::ToTwist2d(WheelPositions const& start, WheelPositions const& end) const {
    WheelPositions result{wpi::empty_array};

    result[0] = {end[0].distance - start[0].distance, end[0].angle};
    result[1] = {end[1].distance - start[1].distance, end[1].angle};
    result[2] = {end[2].distance - start[2].distance, end[2].angle};
    result[3] = {end[3].distance - start[3].distance, end[3].angle};

    return ToTwist2d(result);
}

SwerveDriveKinematics::WheelPositions SwerveDriveKinematics::Interpolate(WheelPositions const& start, WheelPositions const& end, double t) const {
    WheelPositions result{wpi::empty_array};

    result[0] = start[0].Interpolate(end[0], t);
    result[1] = start[1].Interpolate(end[1], t);
    result[2] = start[2].Interpolate(end[2], t);
    result[3] = start[3].Interpolate(end[3], t);

    return result;
}

void SwerveDriveKinematics::DesaturateWheelSpeeds(WheelSpeeds* moduleStates, units::meters_per_second_t attainableMaxSpeed) {
    WheelSpeeds& states = *moduleStates;
    units::meters_per_second_t realMaxSpeed = units::math::abs(std::max_element(states.begin(), states.end(), [](const auto& a, const auto& b) {
                                                                   return units::math::abs(a.speed) < units::math::abs(b.speed);
                                                               })->speed);

    if (realMaxSpeed > attainableMaxSpeed) {
        states[0].speed = states[0].speed / realMaxSpeed * attainableMaxSpeed;
        states[1].speed = states[1].speed / realMaxSpeed * attainableMaxSpeed;
        states[2].speed = states[2].speed / realMaxSpeed * attainableMaxSpeed;
        states[3].speed = states[3].speed / realMaxSpeed * attainableMaxSpeed;
    }
}

void SwerveDriveKinematics::DesaturateWheelSpeeds(WheelSpeeds* moduleStates, frc::ChassisSpeeds desiredChassisSpeed,
                                                  units::meters_per_second_t attainableMaxModuleSpeed,
                                                  units::meters_per_second_t attainableMaxRobotTranslationSpeed,
                                                  units::radians_per_second_t attainableMaxRobotRotationSpeed) {
    WheelSpeeds& states = *moduleStates;
    units::meters_per_second_t realMaxSpeed = units::math::abs(std::max_element(states.begin(), states.end(), [](const auto& a, const auto& b) {
                                                                   return units::math::abs(a.speed) < units::math::abs(b.speed);
                                                               })->speed);

    if (attainableMaxRobotTranslationSpeed == 0_mps || attainableMaxRobotRotationSpeed == 0_rad_per_s || realMaxSpeed == 0_mps) {
        return;
    }

    units::dimensionless::scalar_t translationalK = units::math::hypot(desiredChassisSpeed.vx, desiredChassisSpeed.vy) / attainableMaxRobotTranslationSpeed;

    units::dimensionless::scalar_t rotationalK = units::math::abs(desiredChassisSpeed.omega) / attainableMaxRobotRotationSpeed;

    units::dimensionless::scalar_t k = units::math::max(translationalK, rotationalK);

    units::dimensionless::scalar_t scale = units::math::min(k * attainableMaxModuleSpeed / realMaxSpeed, units::scalar_t{1});

    states[0].speed = states[0].speed * scale;
    states[1].speed = states[1].speed * scale;
    states[2].speed = states[2].speed * scale;
    states[3].speed = states[3].speed * scale;
}