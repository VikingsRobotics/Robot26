#pragma once
#ifndef SUBSYSTEM_MODULE_CONTROL_H
#define SUBSYSTEM_MODULE_CONTROL_H

#include <units/torque.h>
#include <units/current.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <frc/kinematics/SwerveModuleState.h>

/**
 * @brief Control descriptor for a swerve module azimuth (steering) motor.
 *
 * This class describes the desired control mode and target rotation
 * for the azimuth motor. It is a lightweight configuration object
 * intended to be constructed and modified fluently via the With* methods.
 */
class AzimuthModuleControl {
public:
    /**
     * @brief Azimuth control modes.
     */
    enum Mode {
        /// Motor braking / neutral mode.
        kBrake,

        /// Closed-loop position control.
        kPosition,

        /// Closed-loop velocity control.
        kVelocity,

        /// Motion profiling (e.g. MAX Motion).
        kMAXMotionPosition,

        /// Motion profiling (e.g. MAX Motion).
        kMAXMotionVelocity
    };

    /// Selected control mode.
    Mode mode{Mode::kPosition};

    /// Target rotation of the module (in turns).
    units::turn_t rotationTarget;

    /// Target rotation per second of the module (in turns).
    units::turns_per_second_t speedTarget;

public:
    /// Sets brake mode.
    AzimuthModuleControl& WithBrake() {
        mode = Mode::kBrake;
        return *this;
    }
    /// Sets closed-loop position mode with a rotation target.
    AzimuthModuleControl& WithPosition(units::turn_t targetRotation) {
        mode = Mode::kPosition;
        rotationTarget = targetRotation;
        return *this;
    }
    /// Sets closed-loop velocity mode with a speed target.
    AzimuthModuleControl& WithVelocity(units::turns_per_second_t targetSpeed) {
        mode = Mode::kVelocity;
        speedTarget = targetSpeed;
        return *this;
    }
    /// Sets motion-profiled control mode with a rotation target.
    AzimuthModuleControl& WithMAXMotionPosition(units::turn_t targetRotation) {
        mode = Mode::kMAXMotionPosition;
        rotationTarget = targetRotation;
        return *this;
    }
    /// Sets motion-profiled control mode with a speed target.
    AzimuthModuleControl& WithMAXMotionVelocity(units::turns_per_second_t targetSpeed) {
        mode = Mode::kMAXMotionVelocity;
        speedTarget = targetSpeed;
        return *this;
    }
    /// Sets closed-loop position mode.
    AzimuthModuleControl& WithPositionMode() {
        mode = Mode::kPosition;
        return *this;
    }
    /// Sets closed-loop velocity mode.
    AzimuthModuleControl& WithvelocityMode() {
        mode = Mode::kVelocity;
        return *this;
    }
    /// Sets motion-profiled control mode.
    AzimuthModuleControl& WithMAXMotionPositionMode() {
        mode = Mode::kMAXMotionPosition;
        return *this;
    }
    /// Sets motion-profiled control mode.
    AzimuthModuleControl& WithMAXMotionvelocityMode() {
        mode = Mode::kMAXMotionVelocity;
        return *this;
    }
    /// Updates the rotation target without modifying the mode.
    AzimuthModuleControl& WithPositionTarget(units::turn_t targetRotation) {
        rotationTarget = targetRotation;
        return *this;
    }
    /// Updates the speed target without modifying the mode.
    AzimuthModuleControl& WithVelocityTarget(units::turns_per_second_t targetSpeed) {
        speedTarget = targetSpeed;
        return *this;
    }
};


/**
 * @brief Control descriptor for a swerve module drive motor.
 *
 * This class encapsulates:
 * - Feedback control state
 * - Feedforward control state
 * - Output type selection
 * - Target type selection
 * - Feedforward source selection
 *
 */
class DriveModuleControl {
public:
    /**
     * @brief Combined feedforward / feedback control mode flags.
     *
     * Mode is implemented as a 2-bit mask:
     *
     * Bit 0 (LSB) → External Feedforward Enable
     *   0 = No external feedforward applied
     *   1 = External feedforward applied
     *
     * Bit 1 → Feedback Enable
     *   0 = No feedback control (open-loop)
     *   1 = Feedback control active
     *
     * Resulting modes:
     *
     *  kBrake      (0b00)
     *      - No feedback
     *      - No external feedforward
     *      - Motor output is neutral / braking.
     *
     *  kOpenLoop   (0b01)
     *      - No feedback
     *      - External feedforward enabled
     *      - Output is directly driven (duty/voltage/torque).
     *
     *  kClosedLoop (0b10)
     *      - Feedback enabled
     *      - No external feedforward
     *      - Uses onboard controller feedback (PID).
     *      - Internal model feedforwards (kS/kV) may still be used
     *        by the motor controller depending on configuration.
     *      - kA is only used when the selected TargetType supports
     *        motion profiling (e.g., MotionMagicVelocity).
     *
     *  kHybridLoop (0b11)
     *      - Feedback enabled
     *      - External feedforward enabled
     *      - Combines closed-loop control with externally supplied
     *        feedforward terms (e.g., PathPlanner acceleration or forces).
     *      - External feedforward values are interpreted according
     *        to FeedforwardOrigin.
     *
     */
    enum class Mode {

        /// Neutral / brake mode (0b00)
        kBrake = 0b00,

        /// Open-loop control with external feedforward (0b01)
        kOpenLoop = 0b01,

        /// Closed-loop feedback control (0b10)
        kClosedLoop = 0b10,

        /// Closed-loop + external feedforward (0b11)
        kHybridLoop = 0b11
    };


    /**
     * @brief Output type for open-loop or hybrid operation.
     */
    enum class OutputType {
        /// Raw duty cycle output (-1 to 1).
        kDuty,

        /// Voltage output.
        kVoltage,

        /// Torque-producing current output.
        kTorque
    };

    /**
     * @brief Target interpretation for closed-loop modes.
     */
    enum class TargetType {
        /// Distance in motor wheel output
        kPosition,

        /// Velocity in motor wheel output
        kVelocity,

        /// Distance in motor wheel output using motion profile
        kMotionMagicPosition,

        /// Velocity in motor wheel output using motion profile
        kMotionMagicVelocity
    };

    /**
     * @brief Defines how feedforward inputs are interpreted.
     */
    enum class FeedforwardOrigin {
        /// Robot-relative X/Y axis forces.
        kAxis,

        /// Linear force along module direction.
        kLinear,

        /// Acceleration-based feedforward.
        kAcceleration
    };

    /// Active feedforward/feedback mode.
    Mode mode{Mode::kClosedLoop};

    /// Output type for open-loop control.
    OutputType outputType{OutputType::kVoltage};

    /// Active target interpretation.
    TargetType targetType{TargetType::kVelocity};

    /// Feedforward origin interpretation.
    FeedforwardOrigin ffOrigin{FeedforwardOrigin::kAxis};

    /**
     * @brief Closed-loop targets.
     */
    struct Target {
        /// Position target in meters.
        units::meter_t position{0_m};

        /// Velocity target in meters per second.
        units::meters_per_second_t velocity{0_mps};
    } close;

    /**
     * @brief Open-loop output values.
     *
     * Only one field is used depending on OutputType.
     */
    struct OpenLoop {
        /// Duty cycle (-1 to 1).
        units::dimensionless::scalar_t duty{0.0};

        /// Voltage command.
        units::volt_t voltage{0_V};

        /// Torque-producing current.
        units::ampere_t torqueCurrent{0_A};
    } open;

    /**
     * @brief Feedforward parameters.
     *
     * Interpretation depends on FeedforwardOrigin.
     */
    struct Feedforward {
        /// Desired linear acceleration.
        units::meters_per_second_squared_t accel;

        /// Robot-relative X-axis force.
        units::newton_t robotRelativeX;

        /// Robot-relative Y-axis force.
        units::newton_t robotRelativeY;

        /// Linear force along module direction.
        units::newton_t linearForces;
    } ff;

public:
    /// Clears feedback bit (bit 1).
    DriveModuleControl& WithoutFeedback() {
        mode = static_cast<Mode>(static_cast<int>(mode) & ~0b10);
        return *this;
    }
    /// Clears feedforward bit (bit 0).
    DriveModuleControl& WithoutFeedforward() {
        mode = static_cast<Mode>(static_cast<int>(mode) & ~0b01);
        return *this;
    }
    /// Sets feedback bit (bit 1).
    DriveModuleControl& WithFeedback() {
        mode = static_cast<Mode>(static_cast<int>(mode) | 0b10);
        return *this;
    }
    /// Sets feedforward bit (bit 0).
    DriveModuleControl& WithFeedforward() {
        mode = static_cast<Mode>(static_cast<int>(mode) | 0b01);
        return *this;
    }
    /// Sets mode to 0b00 (no feedback, no feedforward).
    DriveModuleControl& WithBrake() {
        mode = Mode::kBrake;
        return *this;
    }
    /// Sets mode to 0b01 (feedforward only).
    DriveModuleControl& WithOpenLoop() {
        mode = Mode::kOpenLoop;
        return *this;
    }
    /// Sets mode to 0b10 (feedback only).
    DriveModuleControl& WithClosedLoop() {
        mode = Mode::kClosedLoop;
        return *this;
    }
    /// Sets mode to 0b11 (feedback + feedforward).
    DriveModuleControl& WithHybridLoop() {
        mode = Mode::kHybridLoop;
        return *this;
    }
    /// Sets the mode to duty cycle output.
    DriveModuleControl& WithDutyCycleMode() {
        outputType = OutputType::kDuty;
        return *this;
    }
    /// Sets the mode to voltage output.
    DriveModuleControl& WithVoltageMode() {
        outputType = OutputType::kVoltage;
        return *this;
    }
    /// Sets the mode to torque output.
    DriveModuleControl& WithTorqueMode() {
        outputType = OutputType::kTorque;
        return *this;
    }
    /// Sets duty cycle output and selects kDuty (open-loop).
    DriveModuleControl& WithDutyCycle(units::dimensionless::scalar_t output) {
        open.duty = output;
        outputType = OutputType::kDuty;
        return *this;
    }
    /// Sets voltage output and selects kVoltage (open-loop).
    DriveModuleControl& WithVoltage(units::volt_t output) {
        open.voltage = output;
        outputType = OutputType::kVoltage;
        return *this;
    }
    /// Sets torque current output and selects kTorque (open-loop).
    DriveModuleControl& WithTorque(units::ampere_t output) {
        open.torqueCurrent = output;
        outputType = OutputType::kTorque;
        return *this;
    }
    /// Sets position target (closed-loop).
    DriveModuleControl& WithPositionTarget(units::meter_t target) {
        close.position = target;
        targetType = TargetType::kPosition;
        return *this;
    }
    /// Sets velocity target (closed-loop).
    DriveModuleControl& WithVelocityTarget(units::meters_per_second_t target) {
        close.velocity = target;
        targetType = TargetType::kVelocity;
        return *this;
    }
    /// Sets motion-profiled position target.
    DriveModuleControl& WithPositionMagic(units::meter_t magicTarget) {
        close.position = magicTarget;
        targetType = TargetType::kMotionMagicPosition;
        return *this;
    }
    /// Sets motion-profiled velocity target.
    DriveModuleControl& WithVelocityMagic(units::meters_per_second_t magicTarget) {
        close.velocity = magicTarget;
        targetType = TargetType::kMotionMagicVelocity;
        return *this;
    }
    /// Sets robot-relative X/Y force feedforward.
    DriveModuleControl& WithAxisFeedforward(units::newton_t x, units::newton_t y) {
        ffOrigin = FeedforwardOrigin::kAxis;
        ff.robotRelativeX = x;
        ff.robotRelativeY = y;
        return *this;
    }
    /// Sets linear force feedforward.
    DriveModuleControl& WithLinearFeedforward(units::newton_t force) {
        ffOrigin = FeedforwardOrigin::kLinear;
        ff.linearForces = force;
        return *this;
    }
    /// Sets acceleration-based feedforward.
    DriveModuleControl& WithAccelerationFeedforward(units::meters_per_second_squared_t acceleration) {
        ffOrigin = FeedforwardOrigin::kAcceleration;
        ff.accel = acceleration;
        return *this;
    }
};


/**
 * @brief High-level control container for a complete swerve module.
 *
 * Combines:
 * - Azimuth (steering) control
 * - Drive (wheel) control
 *
 * Intended to be passed as a single command structure to a swerve module.
 */
class SwerveModuleControl {
public:
    /// Steering motor control.
    AzimuthModuleControl azimuth;

    /// Drive motor control.
    DriveModuleControl drive;

public:
    /// Sets azimuth mode to brake.
    SwerveModuleControl& WithAzimuthBrake() {
        azimuth.WithBrake();
        return *this;
    }
    /// Sets azimuth to closed-loop position with a target rotation.
    SwerveModuleControl& WithAzimuthPosition(units::turn_t rotation) {
        azimuth.WithPosition(rotation);
        return *this;
    }
    /// Sets azimuth to closed-loop position with a speed rotation.
    SwerveModuleControl& WithAzimuthVelocity(units::turns_per_second_t speed) {
        azimuth.WithVelocity(speed);
        return *this;
    }
    /// Sets azimuth to motion-profiled rotation target.
    SwerveModuleControl& WithAzimuthMAXMotionPosition(units::turn_t rotation) {
        azimuth.WithMAXMotionPosition(rotation);
        return *this;
    }
    /// Sets azimuth to motion-profiled speed target.
    SwerveModuleControl& WithAzimuthMAXMotionVelocity(units::turns_per_second_t speed) {
        azimuth.WithMAXMotionVelocity(speed);
        return *this;
    }
    /// Sets azimuth to closed-loop position mode.
    SwerveModuleControl& WithAzimuthPositionMode() {
        azimuth.WithPositionMode();
        return *this;
    }
    /// Sets azimuth to closed-loop position mode.
    SwerveModuleControl& WithAzimuthVelocityMode() {
        azimuth.WithVelocity();
        return *this;
    }
    /// Sets azimuth to motion-profiled control mode.
    SwerveModuleControl& WithAzimuthMAXMotionPositionMode() {
        azimuth.WithMAXMotionPositionMode();
        return *this;
    }
    /// Sets azimuth to motion-profiled control mode.
    SwerveModuleControl& WithAzimuthMAXMotionVelocityMode() {
        azimuth.WithMAXMotionVelocityMode();
        return *this;
    }
    /// Updates azimuth target rotation without changing mode.
    SwerveModuleControl& WithAzimuthPositionTarget(units::turn_t rotation) {
        azimuth.WithPositionTarget(rotation);
        return *this;
    }
    /// Updates azimuth target speed without changing mode.
    SwerveModuleControl& WithAzimuthVelocityTarget(units::turns_per_second_t speed) {
        azimuth.WithVelocityTarget(speed);
        return *this;
    }
    /// Sets drive mode to brake.
    SwerveModuleControl& WithDriveBrake() {
        drive.WithBrake();
        return *this;
    }
    /// Sets drive to open-loop feedforward only.
    SwerveModuleControl& WithDriveOpenLoop() {
        drive.WithOpenLoop();
        return *this;
    }
    /// Sets drive to closed-loop feedback only.
    SwerveModuleControl& WithDriveClosedLoop() {
        drive.WithClosedLoop();
        return *this;
    }
    /// Sets drive to hybrid feedback + feedforward mode.
    SwerveModuleControl& WithDriveHybridLoop() {
        drive.WithHybridLoop();
        return *this;
    }
    /// Sets drive output mode to duty cycle.
    SwerveModuleControl& WithDriveDutyCycleMode() {
        drive.WithDutyCycleMode();
        return *this;
    }
    /// Sets drive output mode to voltage.
    SwerveModuleControl& WithDriveVoltageMode() {
        drive.WithVoltageMode();
        return *this;
    }
    /// Sets drive output mode to torque.
    SwerveModuleControl& WithDriveTorqueMode() {
        drive.WithTorqueMode();
        return *this;
    }
    /// Enables feedback control for drive.
    SwerveModuleControl& WithDriveFeedback() {
        drive.WithFeedback();
        return *this;
    }
    /// Disables feedback control for drive.
    SwerveModuleControl& WithoutDriveFeedback() {
        drive.WithoutFeedback();
        return *this;
    }
    /// Enables feedforward control for drive.
    SwerveModuleControl& WithDriveFeedforward() {
        drive.WithFeedforward();
        return *this;
    }
    /// Disables feedforward control for drive.
    SwerveModuleControl& WithoutDriveFeedforward() {
        drive.WithoutFeedforward();
        return *this;
    }
    /// Sets drive duty cycle output and selects kDuty.
    SwerveModuleControl& WithDriveDutyCycle(units::dimensionless::scalar_t output) {
        drive.WithDutyCycle(output);
        return *this;
    }
    /// Sets drive voltage output and selects kVoltage.
    SwerveModuleControl& WithDriveVoltage(units::volt_t output) {
        drive.WithVoltage(output);
        return *this;
    }
    /// Sets drive torque output and selects kTorque.
    SwerveModuleControl& WithDriveTorque(units::ampere_t output) {
        drive.WithTorque(output);
        return *this;
    }
    /// Sets drive position target (closed-loop).
    SwerveModuleControl& WithDrivePositionTarget(units::meter_t target) {
        drive.WithPositionTarget(target);
        return *this;
    }
    /// Sets drive velocity target (closed-loop).
    SwerveModuleControl& WithDriveVelocityTarget(units::meters_per_second_t target) {
        drive.WithVelocityTarget(target);
        return *this;
    }
    /// Sets motion-profiled drive position target.
    SwerveModuleControl& WithDrivePositionMagic(units::meter_t target) {
        drive.WithPositionMagic(target);
        return *this;
    }
    /// Sets motion-profiled drive velocity target.
    SwerveModuleControl& WithDriveVelocityMagic(units::meters_per_second_t target) {
        drive.WithVelocityMagic(target);
        return *this;
    }
    /// Sets robot-relative X/Y force feedforward for drive.
    SwerveModuleControl& WithDriveAxisFeedforward(units::newton_t x, units::newton_t y) {
        drive.WithAxisFeedforward(x, y);
        return *this;
    }
    /// Sets linear force feedforward for drive.
    SwerveModuleControl& WithDriveLinearFeedforward(units::newton_t force) {
        drive.WithLinearFeedforward(force);
        return *this;
    }
    /// Sets acceleration-based feedforward for drive.
    SwerveModuleControl& WithDriveAccelerationFeedforward(units::meters_per_second_squared_t acceleration) {
        drive.WithAccelerationFeedforward(acceleration);
        return *this;
    }
};

#endif
