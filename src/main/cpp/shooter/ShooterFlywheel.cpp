#include "shooter/ShooterFlywheel.hpp"

#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

static void ConfigureFlywheelFromConstants(rev::spark::SparkFlex& motor, const ShooterFlywheelConstants& configs) {
    SparkFlexConfig flywheelConfig{};

    flywheelConfig.Apply(SparkBaseConfig::Presets::REV_Vortex());
    flywheelConfig.Inverted(configs.MotorInverted);
    flywheelConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    flywheelConfig.SecondaryCurrentLimit(configs.SlipCurrent());

    EncoderConfig flywheelEncoderConfig;

    flywheelEncoderConfig.PositionConversionFactor(((configs.WheelRadius * 2 * std::numbers::pi) / (configs.MotorGearRatio * 1_tr)).value());
    flywheelEncoderConfig.VelocityConversionFactor(((configs.WheelRadius * 2 * std::numbers::pi) / (configs.MotorGearRatio * 1_tr)).value() / 60.0);

    ClosedLoopConfig flywheelClosedLoopConfig;
    flywheelClosedLoopConfig.SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder)
        .PositionWrappingEnabled(configs.MotorGains.posWrapEnabled)
        .PositionWrappingInputRange(configs.MotorGains.posMinInput, configs.MotorGains.posMaxInput)
        .MinOutput(configs.MotorGains.minOut)
        .MaxOutput(configs.MotorGains.maxOut);
    flywheelClosedLoopConfig.maxMotion.CruiseVelocity(configs.MotorGains.cruiseVelocity, kSlot0)
        .MaxAcceleration(configs.MotorGains.maxAcceleration, kSlot0)
        .PositionMode(MAXMotionConfig::MAXMotionPositionMode::kMAXMotionTrapezoidal)
        .AllowedProfileError(configs.MotorGains.allowedError, kSlot0);
    flywheelClosedLoopConfig.Pid(configs.MotorGains.kP, configs.MotorGains.kI, configs.MotorGains.kD, kSlot0)
        .DFilter(configs.MotorGains.dFilter)
        .IZone(configs.MotorGains.iZone)
        .IMaxAccum(configs.MotorGains.iMaxAccum);
    flywheelClosedLoopConfig.feedForward.sva(configs.MotorGains.kS, configs.MotorGains.kV, configs.MotorGains.kA, kSlot0);

    SignalsConfig flywheelSignalConfig;

    flywheelSignalConfig.SetpointAlwaysOn(true).SetpointPeriodMs(20_ms());
    flywheelSignalConfig.PrimaryEncoderVelocityAlwaysOn(true).PrimaryEncoderVelocityPeriodMs(20_ms());

    motor.Configure(flywheelConfig.Apply(flywheelEncoderConfig).Apply(flywheelClosedLoopConfig).Apply(flywheelSignalConfig),
                    rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);
}

static void ConfigureFeederFromConstants(rev::spark::SparkMax& feeder, const ShooterFlywheelConstants& configs) {
    SparkMaxConfig flywheelConfig{};

    flywheelConfig.Apply(SparkBaseConfig::Presets::REV_NEO());
    flywheelConfig.Inverted(configs.MotorInverted);
    flywheelConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

    ClosedLoopConfig flywheelClosedLoopConfig;
    flywheelClosedLoopConfig.SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder)
        .PositionWrappingEnabled(configs.MotorGains.posWrapEnabled)
        .PositionWrappingInputRange(configs.MotorGains.posMinInput, configs.MotorGains.posMaxInput)
        .MinOutput(configs.MotorGains.minOut)
        .MaxOutput(configs.MotorGains.maxOut);
    flywheelClosedLoopConfig.maxMotion.CruiseVelocity(configs.MotorGains.cruiseVelocity, kSlot0)
        .MaxAcceleration(configs.MotorGains.maxAcceleration, kSlot0)
        .PositionMode(MAXMotionConfig::MAXMotionPositionMode::kMAXMotionTrapezoidal)
        .AllowedProfileError(configs.MotorGains.allowedError, kSlot0);
    flywheelClosedLoopConfig.Pid(configs.MotorGains.kP, configs.MotorGains.kI, configs.MotorGains.kD, kSlot0)
        .DFilter(configs.MotorGains.dFilter)
        .IZone(configs.MotorGains.iZone)
        .IMaxAccum(configs.MotorGains.iMaxAccum);
    flywheelClosedLoopConfig.feedForward.sva(configs.MotorGains.kS, configs.MotorGains.kV, configs.MotorGains.kA, kSlot0);

    SignalsConfig flywheelSignalConfig;

    flywheelSignalConfig.SetpointAlwaysOn(true).SetpointPeriodMs(20_ms());
    flywheelSignalConfig.PrimaryEncoderVelocityAlwaysOn(true).PrimaryEncoderVelocityPeriodMs(20_ms());

    feeder.Configure(flywheelConfig.Apply(flywheelClosedLoopConfig).Apply(flywheelSignalConfig), rev::ResetMode::kNoResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
}

ShooterFlywheel::ShooterFlywheel(ShooterFlywheelConstants const& flywheelConstants)
    : flywheelMotor{flywheelConstants.MotorId, rev::spark::SparkLowLevel::MotorType::kBrushless},
      flywheelEncoder{flywheelMotor.GetEncoder()},
      flywheelController{flywheelMotor.GetClosedLoopController()},
      feederMotor{flywheelConstants.FeederId, rev::spark::SparkLowLevel::MotorType::kBrushless},
      feederEncoder{feederMotor.GetEncoder()},
      feederController{feederMotor.GetClosedLoopController()},
      flywheelMotorItems{flywheelMotor, flywheelController, feederMotor, feederController},
      kGearRatio{flywheelConstants.MotorGearRatio},
      kWheelMaxSpeed{(flywheelConstants.SpeedAt12Volts * flywheelConstants.WheelRadius * 2 * std::numbers::pi) / (flywheelConstants.MotorGearRatio * 1_tr)},
      kWheelRadius{flywheelConstants.WheelRadius},
      kWheelMass{flywheelConstants.WheelMass},
      kShooterLocation{flywheelConstants.Location},
      loopFilter{frc::LinearFilter<units::second_t>::MovingAverage(50)} {
    ConfigureFlywheelFromConstants(flywheelMotor, flywheelConstants);
    ConfigureFeederFromConstants(feederMotor, flywheelConstants);
}

void ShooterFlywheel::Periodic() {
    static int32_t failedDaqs = 0;
    static int32_t successfulDaqs = 0;

    const units::second_t now = frc::Timer::GetTimestamp();
    const units::second_t dt = now - lastTimestamp;
    lastTimestamp = now;
    units::second_t averageLoopTime = loopFilter.Calculate(dt);

    requestParameters.kGearRatio = kGearRatio;
    requestParameters.kMaxSurfaceSpeed = kWheelMaxSpeed;
    requestParameters.kWheelRadius = kWheelRadius;
    requestParameters.kWheelMass = kWheelMass;
    requestParameters.shooterPose = cachedRobotPose.TransformBy(kShooterLocation);
    requestParameters.surfaceVelocity = units::meters_per_second_t{flywheelEncoder.GetVelocity()};
    if (flywheelMotor.GetLastError() != rev::REVLibError::kOk) {
        failedDaqs++;
        return;
    }
    requestParameters.timestamp = now;
    requestParameters.updatePeriod = averageLoopTime;

    units::meters_per_second_t desired = -1_mps;

    if (requestToApply) {
        try {
            desired = requestToApply(requestParameters, flywheelMotorItems);
        } catch (const std::exception& e) {
            fmt::print("Flywheel request error: {}\n", e.what());
        }
    }

    cachedState.Velocity = units::meters_per_second_t{flywheelEncoder.GetVelocity()};
    if (flywheelMotor.GetLastError() != rev::REVLibError::kOk) {
        failedDaqs++;
        return;
    }
    cachedState.TargetVelocity = desired;
    cachedState.AppliedOutput = units::dimensionless::scalar_t{flywheelMotor.GetAppliedOutput()};
    if (flywheelMotor.GetLastError() != rev::REVLibError::kOk) {
        failedDaqs++;
        return;
    }
    cachedState.Voltage = units::volt_t{flywheelMotor.GetBusVoltage() * cachedState.AppliedOutput()};
    if (flywheelMotor.GetLastError() != rev::REVLibError::kOk) {
        failedDaqs++;
        return;
    }
    cachedState.Current = units::ampere_t{flywheelMotor.GetOutputCurrent()};
    if (flywheelMotor.GetLastError() != rev::REVLibError::kOk) {
        failedDaqs++;
        return;
    }
    successfulDaqs++;
    cachedState.Timestamp = now;
    cachedState.OdometryPeriod = averageLoopTime;
    cachedState.SuccessfulDaqs = successfulDaqs;
    cachedState.FailedDaqs = failedDaqs;

    if (telemetryFunction) {
        try {
            telemetryFunction(cachedState);
        } catch (const std::exception& e) {
            fmt::print("Flywheel Telemetry error: {}\n", e.what());
        }
    }
}