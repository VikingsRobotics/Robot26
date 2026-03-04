#include "swerve/SwerveDrivetrain.hpp"
#include <frc/Timer.h>

SwerveDrivetrain::OdometryThread::OdometryThread(SwerveDrivetrain& swerveDrivetrain)
    : drivetrain{&swerveDrivetrain},
      thread{},
      threadMtx{},
      allSignals{{
          &drivetrain->modules[0]->drivePosition,
          &drivetrain->modules[0]->driveVelocity,
          &drivetrain->modules[1]->drivePosition,
          &drivetrain->modules[1]->driveVelocity,
          &drivetrain->modules[2]->drivePosition,
          &drivetrain->modules[2]->driveVelocity,
          &drivetrain->modules[3]->drivePosition,
          &drivetrain->modules[3]->driveVelocity,
          &drivetrain->pigeonW,
          &drivetrain->pigeonX,
          &drivetrain->pigeonY,
          &drivetrain->pigeonZ,
      }},
      averageLoopTime{1 / drivetrain->updateFrequency} {
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, drivetrain->pigeonW, drivetrain->pigeonX, drivetrain->pigeonY, drivetrain->pigeonZ);
}

void SwerveDrivetrain::OdometryThread::Run() {
    frc::LinearFilter<units::second_t> loopFilter = frc::LinearFilter<units::second_t>::MovingAverage(50);
    const units::second_t nominalPeriod = 1.0 / drivetrain->updateFrequency;
    units::second_t lastTimestamp = frc::Timer::GetFPGATimestamp();

    while (isRunning.load()) {
        // Wait for all CTRE signals
        if (!ctre::phoenix6::BaseStatusSignal::WaitForAll(nominalPeriod, allSignals).IsOK()) {
            failedDaqs++;
            continue;
        }
        successfulDaqs++;

        // ---- LOCK ACQUIRED HERE ----
        std::scoped_lock lock(drivetrain->stateLock);

        const units::second_t now = frc::Timer::GetFPGATimestamp();
        const units::second_t dt = now - lastTimestamp;
        lastTimestamp = now;
        averageLoopTime = units::second_t(loopFilter.Calculate(dt));

        frc::Quaternion currentOrientation{drivetrain->pigeonW.GetValue()(), drivetrain->pigeonX.GetValue()(), drivetrain->pigeonY.GetValue()(),
                                           drivetrain->pigeonZ.GetValue()()};

        frc::Rotation3d heading{currentOrientation};

        frc::Pose3d odomPose = drivetrain->odometry.Update(heading, drivetrain->modulePositions);
        frc::ChassisSpeeds speeds = drivetrain->kinematics.ToChassisSpeeds(drivetrain->moduleStates);

        drivetrain->requestParameters.kinematics = &drivetrain->kinematics;
        drivetrain->requestParameters.moduleLocations = drivetrain->moduleLocations;
        drivetrain->requestParameters.kMaxSpeed = std::min({
            drivetrain->modules[0]->kSpeedAt12Volts,
            drivetrain->modules[1]->kSpeedAt12Volts,
            drivetrain->modules[2]->kSpeedAt12Volts,
            drivetrain->modules[3]->kSpeedAt12Volts,
        });
        drivetrain->requestParameters.operatorForwardDirection = drivetrain->operatorForwardDirection;
        drivetrain->requestParameters.currentChassisSpeed = speeds;
        drivetrain->requestParameters.currentPose = odomPose;
        drivetrain->requestParameters.timestamp = units::second_t(now);
        drivetrain->requestParameters.updatePeriod = averageLoopTime;

        if (drivetrain->requestToApply) {
            drivetrain->requestToApply(drivetrain->requestParameters, drivetrain->GetModules());
        }

        wpi::array<frc::SwerveModuleState, 4> targetStates{wpi::empty_array};
        drivetrain->modulePositions[0] = drivetrain->modules[0]->GetPosition(false);
        drivetrain->moduleStates[0] = drivetrain->modules[0]->GetCurrentState();
        targetStates[0] = drivetrain->modules[0]->GetTargetState();
        drivetrain->modulePositions[1] = drivetrain->modules[1]->GetPosition(false);
        drivetrain->moduleStates[1] = drivetrain->modules[1]->GetCurrentState();
        targetStates[1] = drivetrain->modules[1]->GetTargetState();
        drivetrain->modulePositions[2] = drivetrain->modules[2]->GetPosition(false);
        drivetrain->moduleStates[2] = drivetrain->modules[2]->GetCurrentState();
        targetStates[2] = drivetrain->modules[2]->GetTargetState();
        drivetrain->modulePositions[3] = drivetrain->modules[3]->GetPosition(false);
        drivetrain->moduleStates[3] = drivetrain->modules[3]->GetCurrentState();
        targetStates[3] = drivetrain->modules[3]->GetTargetState();


        drivetrain->cachedState.Pose = odomPose;
        drivetrain->cachedState.Speeds = speeds;
        drivetrain->cachedState.ModuleStates = drivetrain->moduleStates;
        drivetrain->cachedState.ModuleTargets = targetStates;
        drivetrain->cachedState.ModulePositions = drivetrain->modulePositions;
        drivetrain->cachedState.RawOrientation = heading;
        drivetrain->cachedState.Timestamp = now;
        drivetrain->cachedState.OdometryPeriod = averageLoopTime;
        drivetrain->cachedState.SuccessfulDaqs = successfulDaqs.load();
        drivetrain->cachedState.FailedDaqs = failedDaqs.load();

        if (drivetrain->telemetryFunction) {
            drivetrain->telemetryFunction(drivetrain->cachedState);
        }
    }
}

SwerveDrivetrain::SwerveDrivetrain(SwerveDrivetrainConstants const& drivetrainConstants, units::hertz_t odometryUpdateFrequency,
                                   std::array<double, 4> const& odometryStandardDeviation, std::array<double, 4> const& visionStandardDeviation,
                                   std::span<SwerveModuleConstants const, 4> swerveModules)
    : canbus{drivetrainConstants.CANBusName},
      pigeon2{drivetrainConstants.Pigeon2Id, canbus},
      pigeonW{pigeon2.GetQuatW()},
      pigeonX{pigeon2.GetQuatX()},
      pigeonY{pigeon2.GetQuatY()},
      pigeonZ{pigeon2.GetQuatZ()},
      modules{std::make_unique<SwerveModule>(swerveModules[0], canbus), std::make_unique<SwerveModule>(swerveModules[1], canbus),
              std::make_unique<SwerveModule>(swerveModules[2], canbus), std::make_unique<SwerveModule>(swerveModules[3], canbus)},
      moduleLocations{modules[0]->GetModuleLocation(), modules[1]->GetModuleLocation(), modules[2]->GetModuleLocation(), modules[3]->GetModuleLocation()},
      modulePositions{
          modules[0]->GetPosition(true),
          modules[1]->GetPosition(true),
          modules[2]->GetPosition(true),
          modules[3]->GetPosition(true),
      },
      moduleStates{modules[0]->GetCurrentState(), modules[1]->GetCurrentState(), modules[2]->GetCurrentState(), modules[3]->GetCurrentState()},
      kinematics{moduleLocations},
      odometry{kinematics,
               frc::Rotation3d{frc::Quaternion{pigeonW.GetValue()(), pigeonX.GetValue()(), pigeonY.GetValue()(), pigeonZ.GetValue()()}},
               modulePositions,
               frc::Pose3d{},
               odometryStandardDeviation,
               visionStandardDeviation},
      updateFrequency{odometryUpdateFrequency},
      odometryThread{std::make_unique<OdometryThread>(*this)} {
    odometryThread->Start();
}