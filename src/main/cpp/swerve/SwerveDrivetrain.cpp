#include "swerve/SwerveDrivetrain.hpp"
#include <frc/Timer.h>

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
      odometryThread{[this]() {
          static frc::LinearFilter<units::second_t> loopFilter = frc::LinearFilter<units::second_t>::MovingAverage(50);
          static units::second_t lastTimestamp = frc::Timer::GetTimestamp();
          static wpi::array<ctre::phoenix6::BaseStatusSignal*, 32> allSignals{
              &modules[0]->drivePosition,
              &modules[0]->driveVelocity,
              &modules[0]->driveAcceleration,
              &modules[0]->driveMotorKT,
              &modules[0]->driveMotorStallCurrent,
              &modules[0]->driveMotorOutputCurrent,
              &modules[0]->driveMotorOutputVoltage,

              &modules[1]->drivePosition,
              &modules[1]->driveVelocity,
              &modules[1]->driveAcceleration,
              &modules[1]->driveMotorKT,
              &modules[1]->driveMotorStallCurrent,
              &modules[1]->driveMotorOutputCurrent,
              &modules[1]->driveMotorOutputVoltage,

              &modules[2]->drivePosition,
              &modules[2]->driveVelocity,
              &modules[2]->driveAcceleration,
              &modules[2]->driveMotorKT,
              &modules[2]->driveMotorStallCurrent,
              &modules[2]->driveMotorOutputCurrent,
              &modules[2]->driveMotorOutputVoltage,

              &modules[3]->drivePosition,
              &modules[3]->driveVelocity,
              &modules[3]->driveAcceleration,
              &modules[3]->driveMotorKT,
              &modules[3]->driveMotorStallCurrent,
              &modules[3]->driveMotorOutputCurrent,
              &modules[3]->driveMotorOutputVoltage,

              &pigeonW,
              &pigeonX,
              &pigeonY,
              &pigeonZ,
          };
          static int32_t failedDaqs = 0;
          static int32_t successfulDaqs = 0;


          if (!ctre::phoenix6::BaseStatusSignal::WaitForAll(0_s, allSignals).IsOK()) {
              failedDaqs++;
              return;
          }
          if (!ctre::phoenix6::BaseStatusSignal::IsAllGood(allSignals)) {
              failedDaqs++;
              return;
          }
          successfulDaqs++;

          std::lock_guard lock(stateLock);

          const units::second_t now = frc::Timer::GetTimestamp();
          const units::second_t dt = now - lastTimestamp;
          lastTimestamp = now;
          units::second_t averageLoopTime = loopFilter.Calculate(dt);

          frc::Quaternion currentOrientation{pigeonW.GetValue()(), pigeonX.GetValue()(), pigeonY.GetValue()(), pigeonZ.GetValue()()};

          frc::Rotation3d heading{currentOrientation};

          frc::Pose3d odomPose = odometry.Update(heading, modulePositions);
          frc::ChassisSpeeds speeds = kinematics.ToChassisSpeeds(moduleStates);

          requestParameters.kinematics = &kinematics;
          requestParameters.moduleLocations = moduleLocations;
          requestParameters.kMaxSpeed = std::min({
              modules[0]->kSpeedAt12Volts,
              modules[1]->kSpeedAt12Volts,
              modules[2]->kSpeedAt12Volts,
              modules[3]->kSpeedAt12Volts,
          });
          requestParameters.operatorForwardDirection = operatorForwardDirection;
          requestParameters.currentChassisSpeed = speeds;
          requestParameters.currentPose = odomPose;
          requestParameters.timestamp = units::second_t(now);
          requestParameters.updatePeriod = averageLoopTime;

          if (requestToApply) {
              try {
                  requestToApply(requestParameters, GetModules());
              } catch (const std::exception& e) {
                  fmt::print("Drive request error: {}\n", e.what());
              }
          }

          wpi::array<frc::SwerveModuleState, 4> targetStates{wpi::empty_array};
          modulePositions[0] = modules[0]->GetPosition(false);
          moduleStates[0] = modules[0]->GetCurrentState();
          targetStates[0] = modules[0]->GetTargetState();
          modulePositions[1] = modules[1]->GetPosition(false);
          moduleStates[1] = modules[1]->GetCurrentState();
          targetStates[1] = modules[1]->GetTargetState();
          modulePositions[2] = modules[2]->GetPosition(false);
          moduleStates[2] = modules[2]->GetCurrentState();
          targetStates[2] = modules[2]->GetTargetState();
          modulePositions[3] = modules[3]->GetPosition(false);
          moduleStates[3] = modules[3]->GetCurrentState();
          targetStates[3] = modules[3]->GetTargetState();


          cachedState.Pose = odomPose;
          cachedState.Speeds = speeds;
          cachedState.ModuleStates = moduleStates;
          cachedState.ModuleTargets = targetStates;
          cachedState.ModulePositions = modulePositions;
          cachedState.RawOrientation = heading;
          cachedState.Timestamp = now;
          cachedState.OdometryPeriod = averageLoopTime;
          cachedState.SuccessfulDaqs = successfulDaqs;
          cachedState.FailedDaqs = failedDaqs;

          if (telemetryFunction) {
              try {
                  telemetryFunction(cachedState);
              } catch (const std::exception& e) {
                  fmt::print("Telemetry error: {}\n", e.what());
              }
          }
      }} {
    if (drivetrainConstants.Pigeon2Configs) {
        pigeon2.GetConfigurator().Apply(*drivetrainConstants.Pigeon2Configs);
    }
    odometryThread.SetHALThreadPriority(true, 70);
    odometryThread.StartPeriodic(updateFrequency);
    odometryThread.SetName("Swerve Odometry Thread");
}
