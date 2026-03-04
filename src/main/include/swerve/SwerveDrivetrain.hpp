#ifndef SWERVE_DRIVETRAIN_H
#define SWERVE_DRIVETRAIN_H
#pragma once

#include "swerve/SwerveModule.hpp"
#include "swerve/SwerveDrivePoseEstimator3d.hpp"
#include "swerve/SwerveDrivetrainConstants.hpp"

#include <atomic>
#include <span>
#include <thread>

class SwerveRequest;

/**
 * \brief Swerve Drive class
 *
 * This class handles the kinematics and odometry.
 */
class SwerveDrivetrain {
public:
    /** \brief Performs swerve module updates in a separate thread to minimize latency. */
    class OdometryThread {
    protected:
        SwerveDrivetrain* drivetrain;

        std::thread thread;
        std::mutex threadMtx;
        std::atomic<bool> isRunning = false;

        wpi::array<ctre::phoenix6::BaseStatusSignal*, 15> allSignals;

        units::second_t averageLoopTime;
        std::atomic<int32_t> successfulDaqs{};
        std::atomic<int32_t> failedDaqs{};

    public:
        OdometryThread(SwerveDrivetrain& swerveDrivetrain);
        ~OdometryThread() { Stop(); }

        /**
         * \brief Starts the odometry thread.
         */
        void Start() {
            std::lock_guard<std::mutex> lock{threadMtx};
            if (!thread.joinable()) {
                isRunning.store(true, std::memory_order_relaxed);
                thread = std::thread{[this] { Run(); }};
            }
        }

        /**
         * \brief Stops the odometry thread.
         */
        void Stop() {
            std::lock_guard<std::mutex> lock{threadMtx};
            if (thread.joinable()) {
                isRunning.store(false, std::memory_order_relaxed);
                thread.join();
            }
        }

        /**
         * \brief Check if the odometry is currently valid.
         *
         * \returns True if odometry is valid
         */
        bool IsOdometryValid() const { return successfulDaqs.load(std::memory_order_relaxed) > 5; }

    protected:
        void Run();
    };

    /**
     * \brief Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    struct SwerveDriveState {
        /** \brief The current pose of the robot */
        frc::Pose3d Pose{};
        /** \brief The current robot-centric velocity */
        frc::ChassisSpeeds Speeds{};
        /** \brief The current module states */
        wpi::array<frc::SwerveModuleState, 4> ModuleStates{wpi::empty_array};
        /** \brief The target module states */
        wpi::array<frc::SwerveModuleState, 4> ModuleTargets{wpi::empty_array};
        /** \brief The current module positions */
        wpi::array<frc::SwerveModulePosition, 4> ModulePositions{wpi::empty_array};
        /** \brief The raw heading of the robot, unaffected by vision updates and odometry resets */
        frc::Rotation3d RawOrientation{};
        /** \brief The timestamp of the state capture, in the timebase of utils#GetCurrentTime() */
        units::second_t Timestamp{};
        /** \brief The measured odometry update period */
        units::second_t OdometryPeriod{};
        /** \brief Number of successful data acquisitions */
        int32_t SuccessfulDaqs{};
        /** \brief Number of failed data acquisitions */
        int32_t FailedDaqs{};
    };

    /**
     * \brief Contains everything the control requests need to calculate the module state.
     */
    struct ControlParameters {
        /** \brief The kinematics object used for control */
        SwerveDriveKinematics* kinematics;
        /** \brief The locations of the swerve modules */
        wpi::array<frc::Translation2d, 4> moduleLocations{wpi::empty_array};
        /** \brief The max speed of the robot at 12 V output */
        units::meters_per_second_t kMaxSpeed;

        /** \brief The forward direction from the operator perspective */
        frc::Rotation3d operatorForwardDirection;
        /** \brief The current robot-centric chassis speeds */
        frc::ChassisSpeeds currentChassisSpeed;
        /** \brief The current pose of the robot */
        frc::Pose3d currentPose;
        /** \brief The timestamp of the current control apply, in the timebase of utils#GetCurrentTime() */
        units::second_t timestamp;
        /** \brief The update period of control apply */
        units::second_t updatePeriod;
    };

    using SwerveRequestFunc = std::function<void(ControlParameters const&, std::span<std::unique_ptr<SwerveModule> const, 4>)>;

private:
    friend class OdometryThread;

    ctre::phoenix6::CANBus canbus;

    ctre::phoenix6::hardware::Pigeon2 pigeon2;
    ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t> pigeonW;
    ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t> pigeonX;
    ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t> pigeonY;
    ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t> pigeonZ;

    wpi::array<std::unique_ptr<SwerveModule>, 4> modules;

    wpi::array<frc::Translation2d, 4> moduleLocations;
    wpi::array<frc::SwerveModulePosition, 4> modulePositions;
    wpi::array<frc::SwerveModuleState, 4> moduleStates;

    SwerveDriveKinematics kinematics;
    SwerveDrivePoseEstimator3d odometry;

    frc::Rotation3d operatorForwardDirection{};

    SwerveRequestFunc requestToApply = [](ControlParameters const&, std::span<std::unique_ptr<SwerveModule> const, 4>) {};
    ControlParameters requestParameters{};

    mutable std::recursive_mutex stateLock{};
    SwerveDriveState cachedState{};
    std::function<void(SwerveDriveState const&)> telemetryFunction{};

    units::hertz_t updateFrequency;

    std::unique_ptr<OdometryThread> odometryThread;

public:
    /**
     * \brief Constructs a SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, which can be accessed through
     * getters in the classes.
     *
     * \param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop.
     * \param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                   in the form [x, y, z, theta]ᵀ, with units in meters
     *                                   and radians
     * \param visionStandardDeviation    The standard deviation for vision calculation
     *                                   in the form [x, y, z, theta]ᵀ, with units in meters
     *                                   and radians
     * \param modules                    Constants for each specific module
     */
    SwerveDrivetrain(SwerveDrivetrainConstants const& drivetrainConstants, units::hertz_t odometryUpdateFrequency,
                     std::array<double, 4> const& odometryStandardDeviation, std::array<double, 4> const& visionStandardDeviation,
                     std::span<SwerveModuleConstants const, 4> swerveModules);

    /**
     * \brief Gets the target odometry update frequency.
     *
     * \returns Target odometry update frequency
     */
    units::hertz_t GetOdometryFrequency() const { return updateFrequency; }

    /**
     * \brief Gets a reference to the odometry thread.
     *
     * \returns Odometry thread
     */
    OdometryThread& GetOdometryThread() { return *odometryThread; }

    /**
     * \brief Check if the odometry is currently valid.
     *
     * \returns True if odometry is valid
     */
    bool IsOdometryValid() const { return odometryThread->IsOdometryValid(); }

    /**
     * \brief Gets a reference to the kinematics used for the drivetrain.
     *
     * \returns Swerve kinematics
     */
    SwerveDriveKinematics& GetKinematics() { return kinematics; }

    /**
     * \brief Applies the specified control request to this swerve drivetrain.
     *
     * This captures the swerve request by reference, so it must live for
     * at least as long as the drivetrain. This can be done by storing the
     * request as a member variable of your drivetrain subsystem or robot.
     *
     * \param request Request to apply
     */
    template <std::derived_from<SwerveRequest> Request>
        requires(!std::is_const_v<Request>)
    void SetControl(Request& request) {
        SetControl([&request](auto const& params, auto modules) mutable { return request.Apply(params, modules); });
    }

    /**
     * \brief Applies the specified control request to this swerve drivetrain.
     *
     * \param request Request to apply
     */
    template <std::derived_from<SwerveRequest> Request>
        requires(!std::is_const_v<Request>)
    void SetControl(Request&& request) {
        SetControl([request = std::move(request)](auto const& params, auto modules) mutable { return request.Apply(params, modules); });
    }

    /**
     * \brief Applies the specified control function to this swerve drivetrain.
     *
     * \param request Request function to apply
     */
    void SetControl(SwerveRequestFunc&& request) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        if (request) {
            requestToApply = std::move(request);
        } else {
            requestToApply = [](auto&, auto) {};
        }
    }

    /**
     * \brief Immediately runs the provided temporary control function.
     *
     * This is used to accelerate non-native swerve requests and
     * can only be called from the odometry thread. Otherwise,
     * SetControl should be used instead.
     *
     * \param request Request function to invoke
     */
    void RunTempRequest(SwerveRequestFunc&& request) const {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        request(requestParameters, modules);
    }

    /**
     * \brief Gets the current state of the swerve drivetrain.
     * This includes information such as the pose estimate,
     * module states, and chassis speeds.
     *
     * \returns Current state of the drivetrain
     */
    SwerveDriveState GetState() const {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        return cachedState;
    }

    /**
     * \brief Register the specified lambda to be executed whenever the SwerveDriveState
     * is updated in the odometry thread.
     *
     * It is imperative that this function is cheap, as it will be executed synchronously
     * with the odometry call; if this takes a long time, it may negatively impact the
     * odometry of this stack.
     *
     * This can also be used for logging data if the function performs logging instead of telemetry.
     * Additionally, the SwerveDriveState object can be cloned and stored for later processing.
     *
     * \param telemetryFunction Function to call for telemetry or logging
     */
    void RegisterTelemetry(std::function<void(SwerveDriveState const&)> telemetryFunction) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        telemetryFunction = std::move(telemetryFunction);
    }

    /**
     * \brief Zero's this swerve drive's odometry entirely.
     *
     * This will zero the entire odometry, and place the robot at 0,0
     */
    void TareEverything() {
        std::lock_guard<std::recursive_mutex> lock{stateLock};

        for (size_t i = 0; i < modules.size(); ++i) {
            modules[i]->ResetPosition();
            modulePositions[i] = modules[i]->GetPosition(true);
        }
        odometry.ResetPosition(frc::Rotation3d{frc::Quaternion{pigeonW.GetValue()(), pigeonX.GetValue()(), pigeonY.GetValue()(), pigeonZ.GetValue()()}},
                               modulePositions, frc::Pose3d{});
        /* We need to update our cached pose immediately to prevent race conditions */
        cachedState.Pose = odometry.GetEstimatedPosition();
    }

    /**
     * \brief Resets the rotation of the robot pose to the given value
     * from the requests#ForwardPerspectiveValue#OperatorPerspective
     * perspective. This makes the current orientation of the robot minus
     * `rotation` the X forward for field-centric maneuvers.
     *
     * This is equivalent to calling ResetRotation with `rotation +
     * GetOperatorForwardDirection()`.
     *
     * \param rotation Rotation to make the current rotation
     */
    void SeedFieldCentric(frc::Rotation3d const& rotation = frc::Rotation3d{}) { ResetRotation(rotation + operatorForwardDirection); }

    /**
     * \brief Resets the pose of the robot. The pose should be from the
     * requests#ForwardPerspectiveValue#BlueAlliance perspective.
     *
     * \param pose Pose to make the current pose
     */
    void ResetPose(frc::Pose3d const& pose) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};

        odometry.ResetPose(pose);
        /* We need to update our cached pose immediately to prevent race conditions */
        cachedState.Pose = odometry.GetEstimatedPosition();
    }

    /**
     * \brief Resets the translation of the robot pose without affecting rotation.
     * The translation should be from the requests#ForwardPerspectiveValue#BlueAlliance
     * perspective.
     *
     * \param translation Translation to make the current translation
     */
    void ResetTranslation(frc::Translation3d const& translation) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};

        odometry.ResetTranslation(translation);
        /* We need to update our cached pose immediately to prevent race conditions */
        cachedState.Pose = odometry.GetEstimatedPosition();
    }

    /**
     * \brief Resets the rotation of the robot pose without affecting translation.
     * The rotation should be from the requests#ForwardPerspectiveValue#BlueAlliance
     * perspective.
     *
     * \param rotation Rotation to make the current rotation
     */
    void ResetRotation(frc::Rotation3d const& rotation) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};

        odometry.ResetRotation(rotation);
        /* We need to update our cached pose immediately to prevent race conditions */
        cachedState.Pose = odometry.GetEstimatedPosition();
    }

    /**
     * \brief Takes the requests#ForwardPerspectiveValue#BlueAlliance perpective
     * direction and treats it as the forward direction for
     * requests#ForwardPerspectiveValue#OperatorPerspective.
     *
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     *
     * This does not change the robot pose, which is in the
     * requests#ForwardPerspectiveValue#BlueAlliance perspective.
     * As a result, the robot pose may need to be reset using ResetPose.
     *
     * \param fieldDirection Heading indicating which direction is forward from
     *                       the requests#ForwardPerspectiveValue#BlueAlliance perspective
     */
    void SetOperatorPerspectiveForward(frc::Rotation3d fieldDirection) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        operatorForwardDirection = fieldDirection;
    }

    /**
     * \brief Returns the requests#ForwardPerspectiveValue#BlueAlliance perpective
     * direction that is treated as the forward direction for
     * requests#ForwardPerspectiveValue#OperatorPerspective.
     *
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     *
     * \returns Heading indicating which direction is forward from
     *          the requests#ForwardPerspectiveValue#BlueAlliance perspective
     */
    frc::Rotation3d GetOperatorForwardDirection() const {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        return operatorForwardDirection;
    }

    /**
     * \brief Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * This method can be called as infrequently as you want, as long as you are
     * calling impl#SwerveDrivePoseEstimator#Update every loop.
     *
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * \param visionRobotPose The pose of the robot as measured by the
     *                        vision camera.
     * \param timestamp       The timestamp of the vision measurement in
     *                        seconds. Note that if you don't use your
     *                        own time source by calling
     *                        impl#SwerveDrivePoseEstimator#UpdateWithTime,
     *                        then you must use a timestamp with an epoch
     *                        since system startup (i.e., the epoch of this
     *                        timestamp is the same epoch as utils#GetCurrentTime).
     *                        This means that you should use utils#GetCurrentTime
     *                        as your time source in this case.
     */
    void AddVisionMeasurement(frc::Pose3d visionRobotPose, units::second_t timestamp) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        odometry.AddVisionMeasurement(visionRobotPose, timestamp);
    }

    /**
     * \brief Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * This method can be called as infrequently as you want, as long as you are
     * calling impl#SwerveDrivePoseEstimator#Update every loop.
     *
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * #SetVisionMeasurementStdDevs or this method.
     *
     * \param visionRobotPose          The pose of the robot as measured by the
     *                                 vision camera.
     * \param timestamp                The timestamp of the vision measurement in
     *                                 seconds. Note that if you don't use your
     *                                 own time source by calling
     *                                 impl#SwerveDrivePoseEstimator#UpdateWithTime,
     *                                 then you must use a timestamp with an epoch
     *                                 since system startup (i.e., the epoch of this
     *                                 timestamp is the same epoch as utils#GetCurrentTime).
     *                                 This means that you should use utils#GetCurrentTime
     *                                 as your time source in this case.
     * \param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position in meters, y position
     *                                 in meters, and heading in radians). Increase
     *                                 these numbers to trust the vision pose
     *                                 measurement less.
     */
    void AddVisionMeasurement(frc::Pose3d visionRobotPose, units::second_t timestamp, std::array<double, 4> const& visionMeasurementStdDevs) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        odometry.AddVisionMeasurement(visionRobotPose, timestamp, visionMeasurementStdDevs);
    }

    /**
     * \brief Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in vision measurements after the autonomous period, or to change
     * trust as distance to a vision target increases.
     *
     * \param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these
     *                                 numbers to trust global measurements from
     *                                 vision less. This matrix is in the form [x,
     *                                 y, theta]ᵀ, with units in meters and radians.
     */
    void SetVisionMeasurementStdDevs(std::array<double, 4> const& visionMeasurementStdDevs) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        odometry.SetVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    /**
     * \brief Sets the pose estimator's trust in robot odometry. This might be used
     * to change trust in odometry after an impact with the wall or traversing a bump.
     *
     * \param stateStdDevs Standard deviations of the pose estimate. Increase these
     *                     numbers to trust your state estimate less. This matrix is
     *                     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    void SetStateStdDevs(std::array<double, 4> const& stateStdDevs) {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        odometry.SetStateStdDevs(stateStdDevs);
    }

    /**
     * \brief Return the pose at a given timestamp, if the buffer is not empty.
     *
     * \param timestamp The pose's timestamp. Note that if you don't use your
     *                  own time source by calling
     *                  impl#SwerveDrivePoseEstimator#UpdateWithTime,
     *                  then you must use a timestamp with an epoch
     *                  since system startup (i.e., the epoch of this
     *                  timestamp is the same epoch as utils#GetCurrentTime).
     *                  This means that you should use utils#GetCurrentTime
     *                  as your time source in this case.
     * \returns The pose at the given timestamp (or std::nullopt if the buffer is
     * empty).
     */
    std::optional<frc::Pose3d> SamplePoseAt(units::second_t timestamp) const {
        std::lock_guard<std::recursive_mutex> lock{stateLock};
        return odometry.SampleAt(timestamp);
    }

    /**
     * \brief Get a reference to the module at the specified index.
     * The index corresponds to the module described in the constructor.
     *
     * \param index Which module to get
     * \returns Reference to SwerveModuleImpl
     */
    SwerveModule& GetModule(size_t index) { return *modules[index]; }

    /**
     * \brief Get a reference to the full array of modules.
     * The indexes correspond to the module described in the constructor.
     *
     * \returns Reference to the SwerveModuleImpl array
     */
    std::span<std::unique_ptr<SwerveModule>, 4> GetModules() { return modules; }

    /**
     * \brief Gets the locations of the swerve modules.
     *
     * \returns Reference to the array of swerve module locations
     */
    wpi::array<frc::Translation2d, 4>& GetModuleLocations() { return moduleLocations; }

    /**
     * \brief Gets this drivetrain's Pigeon 2 reference.
     *
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * \returns This drivetrain's Pigeon 2 reference
     */
    ctre::phoenix6::hardware::core::CorePigeon2& GetPigeon2() { return pigeon2; }
};

#endif