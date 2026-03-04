
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

#include <wpi/array.h>

/**
 * \brief Class that converts a chassis velocity (dx, dy, and dtheta components)
 * into individual module states (speed and angle).
 *
 * The inverse kinematics (converting from a desired chassis velocity to
 * individual module states) uses the relative locations of the modules with
 * respect to the center of rotation. The center of rotation for inverse
 * kinematics is also variable. This means that you can set your set your center
 * of rotation in a corner of the robot to perform special evasion maneuvers.
 *
 * Forward kinematics (converting an array of module states into the overall
 * chassis motion) is performs the exact opposite of what inverse kinematics
 * does. Since this is an overdetermined system (more equations than variables),
 * we use a least-squares approximation.
 *
 * The inverse kinematics: [moduleStates] = [moduleLocations] * [chassisSpeeds]
 * We take the Moore-Penrose pseudoinverse of [moduleLocations] and then
 * multiply by [moduleStates] to get our chassis speeds.
 *
 * Forward kinematics is also used for odometry -- determining the position of
 * the robot on the field using encoders and a gyro.
 */
class SwerveDriveKinematics {
public:
    using WheelSpeeds = wpi::array<frc::SwerveModuleState, 4>;
    using WheelPositions = wpi::array<frc::SwerveModulePosition, 4>;

public:
    /**
     * \brief Constructs a swerve drive kinematics object. This takes in a variable
     * number of module locations as Translation2ds. The order in which you pass
     * in the module locations is the same order that you will receive the module
     * states when performing inverse kinematics. It is also expected that you
     * pass in the module states in the same order when calling the forward
     * kinematics methods.
     *
     * \param moduleLocations The locations of the modules relative to the
     *                        physical center of the robot.
     */
    SwerveDriveKinematics(wpi::array<frc::Translation2d, 4> moduleLocations);

    /**
     * \brief Reset the internal swerve module headings.
     *
     * \param moduleHeadings The swerve module headings. The order of the module
     * headings should be same as passed into the constructor of this class.
     */
    void ResetHeadings(wpi::array<frc::Rotation2d, 4> moduleHeadings);

    /**
     * \brief Performs inverse kinematics to return the module states from a desired
     * chassis velocity. This method is often used to convert joystick values into
     * module speeds and angles.
     *
     * This function also supports variable centers of rotation. During normal
     * operations, the center of rotation is usually the same as the physical
     * center of the robot; therefore, the argument is defaulted to that use case.
     * However, if you wish to change the center of rotation for evasive
     * maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * In the case that the desired chassis speeds are zero (i.e. the robot will
     * be stationary), the previously calculated module angle will be maintained.
     *
     * \param chassisSpeeds The desired chassis speed.
     * \param centerOfRotation The center of rotation. For example, if you set the
     * center of rotation at one corner of the robot and provide a chassis speed
     * that only has a dtheta component, the robot will rotate around that corner.
     *
     * \returns A vector containing the module states. Use caution because these
     * module states are not normalized. Sometimes, a user input may cause one of
     * the module speeds to go above the attainable max velocity. Use the
     * DesaturateWheelSpeeds(WheelSpeeds*, units::meters_per_second_t) function to
     * rectify this issue.
     */
    WheelSpeeds ToSwerveModuleStates(frc::ChassisSpeeds const& chassisSpeeds, frc::Translation2d const& centerOfRotation = frc::Translation2d{});

    /**
     * \brief Performs forward kinematics to return the resulting chassis state from the
     * given module states. This method is often used for odometry -- determining
     * the robot's position on the field using data from the real-world speed and
     * angle of each module on the robot.
     *
     * \param moduleStates The state of the modules as a std::vector of type
     * SwerveModuleState as measured from respective encoders and gyros. The
     * order of the swerve module states should be same as passed into the
     * constructor of this class.
     *
     * \returns The resulting chassis speed.
     */
    frc::ChassisSpeeds ToChassisSpeeds(WheelSpeeds const& moduleStates) const;

    /**
     * \brief Performs forward kinematics to return the resulting Twist2d from the
     * given module position deltas. This method is often used for odometry --
     * determining the robot's position on the field using data from the
     * real-world position delta and angle of each module on the robot.
     *
     * \param moduleDeltas The latest change in position of the modules (as a
     * SwerveModulePosition type) as measured from respective encoders and gyros.
     * The order of the swerve module states should be same as passed into the
     * constructor of this class.
     *
     * \returns The resulting Twist2d.
     */
    frc::Twist2d ToTwist2d(WheelPositions const& moduleDeltas) const;

    /**
     * \brief Performs forward kinematics to return the resulting Twist2d from the given
     * change in wheel positions. This method is often used for odometry --
     * determining the robot's position on the field using changes in the distance
     * driven by each wheel on the robot.
     *
     * \param start The starting distances driven by the wheels.
     * \param end The ending distances driven by the wheels.
     *
     * \returns The resulting Twist2d in the robot's movement.
     */
    frc::Twist2d ToTwist2d(WheelPositions const& start, WheelPositions const& end) const;

    /**
     * \brief Performs interpolation between two values.
     *
     * \param start The value to start at.
     * \param end The value to end at.
     * \param t How far between the two values to interpolate. This should be
     * bounded to [0, 1].
     * \returns The interpolated value.
     */
    WheelPositions Interpolate(WheelPositions const& start, WheelPositions const& end, double t) const;

    /**
     * \brief Renormalizes the wheel speeds if any individual speed is above the
     * specified maximum.
     *
     * Sometimes, after inverse kinematics, the requested speed
     * from one or more modules may be above the max attainable speed for the
     * driving motor on that module. To fix this issue, one can reduce all the
     * wheel speeds to make sure that all requested module speeds are at-or-below
     * the absolute threshold, while maintaining the ratio of speeds between
     * modules.
     *
     * \param moduleStates Reference to vector of module states. The vector will be
     * mutated with the normalized speeds!
     * \param attainableMaxSpeed The absolute max speed that a module can reach.
     */
    static void DesaturateWheelSpeeds(WheelSpeeds* moduleStates, units::meters_per_second_t attainableMaxSpeed);

    /**
     * Renormalizes the wheel speeds if any individual speed is above the
     * specified maximum, as well as getting rid of joystick saturation at edges
     * of joystick.
     *
     * Sometimes, after inverse kinematics, the requested speed
     * from one or more modules may be above the max attainable speed for the
     * driving motor on that module. To fix this issue, one can reduce all the
     * wheel speeds to make sure that all requested module speeds are at-or-below
     * the absolute threshold, while maintaining the ratio of speeds between
     * modules.
     *
     * Scaling down the module speeds rotates the direction of net motion in the
     * opposite direction of rotational velocity, which makes discretizing the
     * chassis speeds inaccurate because the discretization did not account for
     * this translational skew.
     *
     * @param moduleStates Reference to array of module states. The array will be
     * mutated with the normalized speeds!
     * @param desiredChassisSpeed The desired speed of the robot
     * @param attainableMaxModuleSpeed The absolute max speed a module can reach
     * @param attainableMaxRobotTranslationSpeed The absolute max speed the robot
     * can reach while translating
     * @param attainableMaxRobotRotationSpeed The absolute max speed the robot can
     * reach while rotating
     */
    static void DesaturateWheelSpeeds(WheelSpeeds* moduleStates, frc::ChassisSpeeds desiredChassisSpeed, units::meters_per_second_t attainableMaxModuleSpeed,
                                      units::meters_per_second_t attainableMaxRobotTranslationSpeed,
                                      units::radians_per_second_t attainableMaxRobotRotationSpeed);

private:
    wpi::array<frc::Translation2d, 4> m_moduleLocations;

    mutable frc::Matrixd<8, 3> m_inverseKinematics;
    Eigen::HouseholderQR<frc::Matrixd<8, 3>> m_forwardKinematics;

    mutable wpi::array<frc::Rotation2d, 4> m_lastModuleHeading;
    frc::Translation2d m_lastCOR{};
};

#endif