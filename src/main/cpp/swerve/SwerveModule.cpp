#include "swerve/SwerveModule.hpp"
#include "swerve/SwerveModuleConstants.hpp"

#include "Constants.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/StatusSignal.hpp>


using namespace ctre::phoenix6;
using namespace rev::spark;

static configs::TalonFXConfiguration GetDriveConfigurationFromConfig(const SwerveModuleConstants& configs) {
    using namespace signals;
    configs::TalonFXConfiguration driveConfig;

    driveConfig.MotorOutput.WithInverted(configs.DriveMotorInverted ? InvertedValue::CounterClockwise_Positive : InvertedValue::Clockwise_Positive)
        .WithNeutralMode(NeutralModeValue::Brake)
        .WithPeakForwardDutyCycle(1.0)
        .WithPeakReverseDutyCycle(-1.0)
        .WithDutyCycleNeutralDeadband(0.0)
        .WithControlTimesyncFreqHz(0_Hz);

    driveConfig.CurrentLimits.WithStatorCurrentLimit(configs.SlipCurrent)
        .WithStatorCurrentLimitEnable(true)
        .WithSupplyCurrentLimit(70_A)
        .WithSupplyCurrentLimitEnable(true)
        .WithSupplyCurrentLowerLimit(40_A)
        .WithSupplyCurrentLowerTime(1.0_s);

    driveConfig.Voltage.WithPeakForwardVoltage(12_V).WithPeakReverseVoltage(-12_V).WithSupplyVoltageTimeConstant(0.0_s);

    driveConfig.TorqueCurrent.WithPeakForwardTorqueCurrent(configs.SlipCurrent)
        .WithPeakReverseTorqueCurrent(-configs.SlipCurrent)
        .WithTorqueNeutralDeadband(0.0_A);

    driveConfig.Feedback.WithFeedbackSensorSource(FeedbackSensorSourceValue::RotorSensor)
        .WithFeedbackRotorOffset(0_tr)
        .WithRotorToSensorRatio(1)
        .WithSensorToMechanismRatio(1)
        .WithVelocityFilterTimeConstant(0.0_s);

    driveConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(0_s).WithVoltageOpenLoopRampPeriod(0_s).WithTorqueOpenLoopRampPeriod(0_s);

    driveConfig.ClosedLoopRamps.WithDutyCycleClosedLoopRampPeriod(0_s).WithVoltageClosedLoopRampPeriod(0_s).WithTorqueClosedLoopRampPeriod(0_s);

    driveConfig.Audio.WithBeepOnBoot(true).WithBeepOnConfig(true).WithAllowMusicDurDisable(true);

    driveConfig.MotionMagic.WithMotionMagicCruiseVelocity(configs.SpeedAt12Volts * (1_tr / 1_m))
        .WithMotionMagicAcceleration((configs.SpeedAt12Volts / 1_s) * (1_tr / 1_m))
        .WithMotionMagicJerk(0_tr_per_s_cu)
        .WithMotionMagicExpo_kV(ctre::unit::volts_per_turn_per_second_t{configs.DriveMotorGains.kV} *
                                ((configs.DriveMotorGearRatio * 1_m) / configs.WheelRadius))
        .WithMotionMagicExpo_kA(ctre::unit::volts_per_turn_per_second_squared_t{configs.DriveMotorGains.kA} *
                                ((configs.DriveMotorGearRatio * 1_m) / configs.WheelRadius));

    driveConfig.Slot0 = configs.DriveMotorGains;

    return driveConfig;
}

static void ConfigureAzimuthFromConstants(rev::spark::SparkMax& motor, const SwerveModuleConstants& configs) {
    SparkMaxConfig azimuthConfig{};

    azimuthConfig.Inverted(configs.SteerMotorInverted);
    azimuthConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

    AbsoluteEncoderConfig azimuthAbsEncoderConfig;

    azimuthAbsEncoderConfig.Apply(AbsoluteEncoderConfig::Presets::REV_ThroughBoreEncoder());
    azimuthAbsEncoderConfig.PositionConversionFactor(1);
    azimuthAbsEncoderConfig.VelocityConversionFactor(1.0 / 60.0);
    azimuthAbsEncoderConfig.ZeroCentered(false);
    azimuthAbsEncoderConfig.Inverted(configs.EncoderInverted);

    ClosedLoopConfig azimuthClosedLoopConfig;
    azimuthClosedLoopConfig.SetFeedbackSensor(FeedbackSensor::kAbsoluteEncoder)
        .PositionWrappingEnabled(configs.SteerMotorGains.posWrapEnabled)
        .PositionWrappingInputRange(configs.SteerMotorGains.posMinInput, configs.SteerMotorGains.posMaxInput)
        .MinOutput(configs.SteerMotorGains.minOut)
        .MaxOutput(configs.SteerMotorGains.maxOut);
    azimuthClosedLoopConfig.maxMotion.CruiseVelocity(configs.SteerMotorGains.cruiseVelocity, kSlot0)
        .MaxAcceleration(configs.SteerMotorGains.maxAcceleration, kSlot0)
        .PositionMode(MAXMotionConfig::MAXMotionPositionMode::kMAXMotionTrapezoidal)
        .AllowedProfileError(configs.SteerMotorGains.allowedError, kSlot0);
    azimuthClosedLoopConfig.Pid(configs.SteerMotorGains.kP, configs.SteerMotorGains.kI, configs.SteerMotorGains.kD, kSlot0)
        .DFilter(configs.SteerMotorGains.dFilter)
        .IZone(configs.SteerMotorGains.iZone)
        .IMaxAccum(configs.SteerMotorGains.iMaxAccum);
    azimuthClosedLoopConfig.feedForward.sva(configs.SteerMotorGains.kS, configs.SteerMotorGains.kV, configs.SteerMotorGains.kA, kSlot0);

    SignalsConfig azimuthSignalConfig;

    azimuthSignalConfig.AbsoluteEncoderPositionAlwaysOn(true).AbsoluteEncoderPositionPeriodMs(20_ms());
    azimuthSignalConfig.AbsoluteEncoderVelocityAlwaysOn(true).AbsoluteEncoderVelocityPeriodMs(20_ms());
    azimuthSignalConfig.MaxMotionSetpointPositionAlwaysOn(true).MaxMotionSetpointPositionPeriodMs(20_ms());
    azimuthSignalConfig.MaxMotionSetpointVelocityAlwaysOn(true).MaxMotionSetpointVelocityPeriodMs(20_ms());
    azimuthSignalConfig.SetpointAlwaysOn(true).SetpointPeriodMs(20_ms());

    motor.Configure(azimuthConfig.Apply(azimuthAbsEncoderConfig).Apply(azimuthClosedLoopConfig).Apply(azimuthSignalConfig),
                    rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);
}


SwerveModule::SwerveModule(const SwerveModuleConstants& constants, ctre::phoenix6::CANBus canBus)
    : driveMotor{constants.DriveMotorId, canBus},
      steerMotor{constants.SteerMotorId, rev::spark::SparkLowLevel::MotorType::kBrushless},
      steerAbsoluteEncoder{steerMotor.GetAbsoluteEncoder()},
      steerLoopController{steerMotor.GetClosedLoopController()},
      chassisAngularOffset{constants.EncoderOffset},
      moduleOffset{constants.LocationX, constants.LocationY},
      drivePosition{driveMotor.GetPosition()},
      driveVelocity{driveMotor.GetVelocity()},
      driveAcceleration{driveMotor.GetAcceleration()},
      driveMotorKT{driveMotor.GetMotorKT()},
      driveMotorStallCurrent{driveMotor.GetMotorStallCurrent()},
      driveMotorOutputCurrent{driveMotor.GetTorqueCurrent()},
      driveMotorOutputVoltage{driveMotor.GetMotorVoltage()},
      moduleSupplem{frc::LinearFilter<units::turns_per_second_t>::SinglePoleIIR(0.25, 20_ms)},
      kDriveRotationsPerMeter{(constants.DriveMotorGearRatio * 1_tr) / constants.WheelRadius},
      kDriveNmPerWheelN{constants.WheelRadius / constants.DriveMotorGearRatio},
      kCouplingRatioDriveRotorToEncoder{constants.CouplingGearRatio},
      kSpeedAt12Volts{constants.SpeedAt12Volts},
      targetState{} {
    driveMotor.GetConfigurator().Apply(GetDriveConfigurationFromConfig(constants));
    ConfigureAzimuthFromConstants(steerMotor, constants);
    UpdateModuleSupplem();
}

void SwerveModule::Apply(ModuleRequest const& moduleRequest) {
    frc::SwerveModuleState desired = moduleRequest.State;
    frc::Rotation2d currentRotation{units::turn_t{steerAbsoluteEncoder.GetPosition()} + chassisAngularOffset};
    desired.Optimize(currentRotation);
    targetState = desired;

    units::turn_t targetRotations = targetState.angle.Radians();

    /**
     * Wait for the day that REV supports putting rotation speed in PID command
     */
    // units::turns_per_second_t rotationSpeed = targetState.angle.Radians() / moduleRequest.UpdatePeriod;

    switch (moduleRequest.SteerRequest) {
        case ModuleSteerRequestType::Position:
            steerLoopController.SetSetpoint(targetRotations(), rev::spark::SparkLowLevel::ControlType::kPosition);
            break;
        case ModuleSteerRequestType::SmartMotion:
            steerLoopController.SetSetpoint(targetRotations(), rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl);
            break;

        default:
            steerMotor.StopMotor();
            break;
    }

    desired.speed = ApplyVelocityCorrections(desired.speed * kDriveRotationsPerMeter, (desired.angle - currentRotation).Radians()) / kDriveRotationsPerMeter;
    MotorTorqueFeedforwards ff = CalculateMotorTorqueFeedforwards(moduleRequest.WheelForceFeedforwardX, moduleRequest.WheelForceFeedforwardY);

    switch (moduleRequest.DriveRequest) {
        case ModuleDriveRequestType::OpenLoopVoltage:
            driveMotor.SetControl(ctre::phoenix6::controls::VoltageOut{(desired.speed / kSpeedAt12Volts) * 12_V});
            break;

        case ModuleDriveRequestType::Velocity:
            driveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(desired.speed * kDriveRotationsPerMeter)}.WithFeedForward(ff.voltage));
            break;

        default:
            driveMotor.SetControl(ctre::phoenix6::controls::StaticBrake{});
            break;
    }

    cachedState = CacheState{.drive = CacheState::Drive{.position = drivePosition.GetValue() / kDriveRotationsPerMeter,
                                                        .velocity = driveVelocity.GetValue() / kDriveRotationsPerMeter,
                                                        .acceleration = driveAcceleration.GetValue() / kDriveRotationsPerMeter,
                                                        .voltage = driveMotorOutputVoltage.GetValue(),
                                                        .current = driveMotorOutputCurrent.GetValue()},
                             .azimuth = CacheState::Azimuth{.position = units::turn_t{steerAbsoluteEncoder.GetPosition()},
                                                            .velocity = units::turns_per_second_t{steerAbsoluteEncoder.GetVelocity()},
                                                            .voltage = units::volt_t{steerMotor.GetBusVoltage() * steerMotor.GetAppliedOutput()},
                                                            .current = units::ampere_t{steerMotor.GetOutputCurrent()}}};
}

frc::SwerveModulePosition SwerveModule::GetPosition(bool refresh) {
    if (refresh) {
        drivePosition.Refresh(false);
    }
    return frc::SwerveModulePosition{drivePosition.GetValue() / kDriveRotationsPerMeter,
                                     frc::Rotation2d{units::turn_t{steerAbsoluteEncoder.GetPosition()} + chassisAngularOffset}};
}

SwerveModule::MotorTorqueFeedforwards SwerveModule::CalculateMotorTorqueFeedforwards(units::newton_t feedforwardX, units::newton_t feedforwardY) const {
    units::newton_t distance = units::math::hypot(feedforwardX, feedforwardY);
    if (distance > 1e-9_N) {
        MotorTorqueFeedforwards calculated;

        frc::Rotation2d feedforwardRotation{feedforwardX.value(), feedforwardY.value()};
        frc::Rotation2d moduleFeedforwardRotation =
            feedforwardRotation.RotateBy(frc::Rotation2d{units::turn_t{steerAbsoluteEncoder.GetPosition()} + chassisAngularOffset});

        calculated.torque = distance * moduleFeedforwardRotation.Cos() * kDriveNmPerWheelN;
        ctre::unit::newton_meters_per_ampere_t motorKT = driveMotorKT.GetValue();
        calculated.torqueCurrent = calculated.torque / motorKT;
        calculated.voltage = (12_V / driveMotorStallCurrent.GetValue()) * calculated.torqueCurrent;

        return calculated;
    }
    return MotorTorqueFeedforwards{units::newton_meter_t{0}, units::ampere_t{0}, units::volt_t{0}};
}

units::turns_per_second_t SwerveModule::ApplyVelocityCorrections(units::turns_per_second_t velocity, units::turn_t angleDifference) const {
    double cosineScale = units::math::cos(angleDifference)();

    velocity = (velocity * cosineScale) - (moduleSupplemSpeed * kCouplingRatioDriveRotorToEncoder);

    return velocity;
}

void SwerveModule::UpdateModuleSupplem() {
    units::turns_per_second_t steerSpeed{steerAbsoluteEncoder.GetVelocity()};
    moduleSupplemSpeed = moduleSupplem.Calculate(steerSpeed);
}