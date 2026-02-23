#include "subsystems/SwerveModule.hpp"

#include "constants/SwerveModuleConfig.hpp"

#include "Constants.hpp"

#include <frc/smartdashboard/SmartDashboard.h>


using namespace ctre::phoenix6;
using namespace rev::spark;

static configs::TalonFXConfiguration GetDriveConfigurationFromConfig(SwerveModuleConfigs& configs) {
    using namespace signals;
    configs::TalonFXConfiguration driveConfig;

    driveConfig.MotorOutput.WithInverted(InvertedValue::CounterClockwise_Positive)
        .WithNeutralMode(NeutralModeValue::Brake)
        .WithPeakForwardDutyCycle(1.0)
        .WithPeakReverseDutyCycle(-1.0)
        .WithDutyCycleNeutralDeadband(0.0)
        .WithControlTimesyncFreqHz(0_Hz);

    driveConfig.CurrentLimits.WithStatorCurrentLimit(120_A)
        .WithStatorCurrentLimitEnable(true)
        .WithSupplyCurrentLimit(70_A)
        .WithSupplyCurrentLimitEnable(true)
        .WithSupplyCurrentLowerLimit(40_A)
        .WithSupplyCurrentLowerTime(1.0_s);

    driveConfig.Voltage.WithPeakForwardVoltage(12_V).WithPeakReverseVoltage(-12_V).WithSupplyVoltageTimeConstant(0.0_s);

    driveConfig.TorqueCurrent.WithPeakForwardTorqueCurrent(800_A).WithPeakReverseTorqueCurrent(-800_A).WithTorqueNeutralDeadband(0.0_A);

    driveConfig.Feedback.WithFeedbackSensorSource(FeedbackSensorSourceValue::RotorSensor)
        .WithFeedbackRotorOffset(0_tr)
        .WithRotorToSensorRatio(1)
        .WithSensorToMechanismRatio(configs.kDriveGearRatio)
        .WithVelocityFilterTimeConstant(0.0_s);

    driveConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(0_s).WithVoltageOpenLoopRampPeriod(0_s).WithTorqueOpenLoopRampPeriod(0_s);

    driveConfig.ClosedLoopRamps.WithDutyCycleClosedLoopRampPeriod(0_s).WithVoltageClosedLoopRampPeriod(0_s).WithTorqueClosedLoopRampPeriod(0_s);

    driveConfig.Audio.WithBeepOnBoot(true).WithBeepOnConfig(true).WithAllowMusicDurDisable(configs.kAllowMusicDuringDisable);

    driveConfig.MotionMagic.WithMotionMagicCruiseVelocity(configs.kMaxDriveSpeed / configs.kGearedRotationToOutputDistance)
        .WithMotionMagicAcceleration(configs.kMaxDriveAccel / configs.kGearedRotationToOutputDistance)
        .WithMotionMagicJerk(0_tr_per_s_cu)
        .WithMotionMagicExpo_kV(configs.kVMagicDrive * configs.kGearedRotationToOutputDistance)
        .WithMotionMagicExpo_kA(configs.kAMagicDrive * configs.kGearedRotationToOutputDistance);

    driveConfig.Slot0.WithGravityType(GravityTypeValue::Elevator_Static)
        .WithStaticFeedforwardSign(StaticFeedforwardSignValue::UseVelocitySign)
        .WithKP(configs.kDriveDutyGain.pid.kP())
        .WithKI(configs.kDriveDutyGain.pid.kI())
        .WithKD(configs.kDriveDutyGain.pid.kD())
        .WithKS(configs.kDriveDutyGain.sva.kS())
        .WithKV(configs.kDriveDutyGain.sva.kV())
        .WithKA(configs.kDriveDutyGain.sva.kA())
        .WithKG(0);

    driveConfig.Slot1.WithGravityType(GravityTypeValue::Elevator_Static)
        .WithStaticFeedforwardSign(StaticFeedforwardSignValue::UseVelocitySign)
        .WithKP(configs.kDriveVoltageGain.pid.kP())
        .WithKI(configs.kDriveVoltageGain.pid.kI())
        .WithKD(configs.kDriveVoltageGain.pid.kD())
        .WithKS(configs.kDriveVoltageGain.sva.kS())
        .WithKV(configs.kDriveVoltageGain.sva.kV())
        .WithKA(configs.kDriveVoltageGain.sva.kA())
        .WithKG(0);

    driveConfig.Slot2.WithGravityType(GravityTypeValue::Elevator_Static)
        .WithStaticFeedforwardSign(StaticFeedforwardSignValue::UseVelocitySign)
        .WithKP(configs.kDriveTorqueGain.pid.kP())
        .WithKI(configs.kDriveTorqueGain.pid.kI())
        .WithKD(configs.kDriveTorqueGain.pid.kD())
        .WithKS(configs.kDriveTorqueGain.sva.kS())
        .WithKV(configs.kDriveTorqueGain.sva.kV())
        .WithKA(configs.kDriveTorqueGain.sva.kA())
        .WithKG(0);

    return driveConfig;
}

static SparkMaxConfig& GetAzimuthConfigurationFromConfig(SwerveModuleConfigs& configs) {
    static SparkMaxConfig azimuthConfig{};

    AbsoluteEncoderConfig azimuthAbsEncoderConfig;

    azimuthAbsEncoderConfig.Apply(AbsoluteEncoderConfig::Presets::REV_ThroughBoreEncoder());
    azimuthAbsEncoderConfig.PositionConversionFactor(configs.kAzimuthRotationToOutputRotation());
    azimuthAbsEncoderConfig.VelocityConversionFactor(configs.kAzimuthRotationToOutputRotation() / 60);
    azimuthAbsEncoderConfig.ZeroCentered(false);
    azimuthAbsEncoderConfig.Inverted(configs.kInvertEncoder);

    ClosedLoopConfig azimuthClosedLoopConfig;
    azimuthClosedLoopConfig.SetFeedbackSensor(FeedbackSensor::kAbsoluteEncoder);
    azimuthClosedLoopConfig.PositionWrappingEnabled(true).PositionWrappingInputRange(0, 1);
    azimuthClosedLoopConfig.MinOutput(-1).MaxOutput(1);
    azimuthClosedLoopConfig.maxMotion.CruiseVelocity(configs.kMaxAzimuthSpeed(), kSlot0)
        .MaxAcceleration(configs.kMaxAzimuthAccel(), kSlot0)
        .PositionMode(MAXMotionConfig::MAXMotionPositionMode::kMAXMotionTrapezoidal);
    azimuthClosedLoopConfig.Pid(configs.kAzimuthPID.kP(), configs.kAzimuthPID.kI(), configs.kAzimuthPID.kD(), kSlot0);
    azimuthClosedLoopConfig.feedForward.sva(configs.kAzimuthFF.kS(), configs.kAzimuthFF.kV(), configs.kAzimuthFF.kA(), kSlot0);

    SignalsConfig azimuthSignalConfig;

    azimuthSignalConfig.AbsoluteEncoderPositionAlwaysOn(true).AbsoluteEncoderPositionPeriodMs(10_ms());
    azimuthSignalConfig.MaxMotionSetpointPositionAlwaysOn(true).MaxMotionSetpointPositionPeriodMs(10_ms());
    azimuthSignalConfig.SetpointAlwaysOn(true).SetpointPeriodMs(10_ms());

    azimuthConfig.Apply(azimuthAbsEncoderConfig).Apply(azimuthClosedLoopConfig).Apply(azimuthSignalConfig);

    return azimuthConfig;
}


SwerveModule::SwerveModule(SwerveModuleConfigs configs)
    : m_drivingTalonFx{configs.kDriveId, DeviceIdentifier::kCANBus},
      m_azimuthSparkMax{configs.kAzimuthId, rev::spark::SparkLowLevel::MotorType::kBrushless},
      m_azimuthAbsoluteEncoder{m_azimuthSparkMax.GetAbsoluteEncoder()},
      m_sparkLoopController{m_azimuthSparkMax.GetClosedLoopController()},
      m_getTalonPosition{m_drivingTalonFx.GetPosition().AsSupplier()},
      m_getTalonVelocity{m_drivingTalonFx.GetVelocity().AsSupplier()},
      m_configs{configs} {
    m_drivingTalonFx.GetConfigurator().Apply(GetDriveConfigurationFromConfig(configs));
    m_azimuthSparkMax.Configure(GetAzimuthConfigurationFromConfig(configs), rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);

    ResetEncoder();
    ApplyControl(SwerveModuleControl{}.WithAzimuthBrake().WithDriveBrake());
}

frc::SwerveModuleState SwerveModule::GetState() {
    return frc::SwerveModuleState{m_getTalonVelocity() * m_configs.kGearedRotationToOutputDistance,
                                  frc::Rotation2d{units::turn_t{m_azimuthAbsoluteEncoder.GetPosition()} - m_configs.kRotationOffset}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return frc::SwerveModulePosition{m_getTalonPosition() * m_configs.kGearedRotationToOutputDistance,
                                     frc::Rotation2d{units::turn_t{m_azimuthAbsoluteEncoder.GetPosition()} - m_configs.kRotationOffset}};
}

ctre::phoenix6::hardware::TalonFX& SwerveModule::GetDriveMotor() {
    return m_drivingTalonFx;
}

rev::spark::SparkMax& SwerveModule::GetAzimuthMotor() {
    return m_azimuthSparkMax;
}

rev::spark::SparkAbsoluteEncoder& SwerveModule::GetAzimuthEncoder() {
    return m_azimuthAbsoluteEncoder;
}

SwerveModuleConfigs SwerveModule::GetConfigs() {
    return m_configs;
}

void SwerveModule::ApplyConfigs(SwerveModuleConfigs configs) {
    m_drivingTalonFx.GetConfigurator().Apply(GetDriveConfigurationFromConfig(configs));
    m_azimuthSparkMax.Configure(GetAzimuthConfigurationFromConfig(configs), rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);

    if (configs.kResetEncoderOnConfig) {
        ResetEncoder();
    }

    m_configs.Apply(configs);
}

void SwerveModule::ResetEncoder() {
    m_drivingTalonFx.SetPosition(0_tr);
}

void SwerveModule::ApplyControl(SwerveModuleControl control) {
    switch (control.drive.mode) {
        case DriveModuleControl::Mode::kBrake:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::StaticBrake{});
            break;
        case DriveModuleControl::Mode::kOpenLoop:
            OpenLoop(control);
            break;
        case DriveModuleControl::Mode::kClosedLoop:
            ClosedLoop(control, Feedforward{0_A, 0_V, units::scalar_t{0}});
            break;
        case DriveModuleControl::Mode::kHybridLoop:
            ClosedLoop(control, CalculateFeedforward(control));
            break;
    }

    switch (control.azimuth.mode) {
        case AzimuthModuleControl::Mode::kBrake:
            m_azimuthSparkMax.StopMotor();
            break;
        case AzimuthModuleControl::Mode::kPosition:
            m_sparkLoopController.SetSetpoint(control.azimuth.rotationTarget(), rev::spark::SparkLowLevel::ControlType::kPosition);
            break;
        case AzimuthModuleControl::Mode::kVelocity:
            m_sparkLoopController.SetSetpoint(control.azimuth.speedTarget(), rev::spark::SparkLowLevel::ControlType::kVelocity);
            break;
        case AzimuthModuleControl::Mode::kMAXMotionPosition:
            m_sparkLoopController.SetSetpoint(control.azimuth.rotationTarget(), rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl);
            break;
        case AzimuthModuleControl::Mode::kMAXMotionVelocity:
            m_sparkLoopController.SetSetpoint(control.azimuth.speedTarget(), rev::spark::SparkLowLevel::ControlType::kMAXMotionVelocityControl);
            break;
    }
}

SwerveModuleControl SwerveModule::GetControl(frc::SwerveModuleState desired, bool cosineLimited) {
    SwerveModuleControl calculatedControl;
    frc::SwerveModuleState optimizedDesiredState{};
    optimizedDesiredState.speed = desired.speed;
    optimizedDesiredState.angle = desired.angle + frc::Rotation2d{m_configs.kRotationOffset};

    frc::Rotation2d currentRotation = frc::Rotation2d{units::turn_t{m_azimuthAbsoluteEncoder.GetPosition()}};

    optimizedDesiredState.Optimize(currentRotation);
    if (cosineLimited) {
        optimizedDesiredState.CosineScale(currentRotation);
    }

    if (std::abs(optimizedDesiredState.speed()) < 0.01) {
        calculatedControl.WithDriveBrake();
    } else {
        calculatedControl.WithDriveVelocityTarget(optimizedDesiredState.speed).WithDriveFeedback();
    }

    calculatedControl.WithAzimuthPositionTarget(optimizedDesiredState.angle.Degrees());

    return calculatedControl;
}

SwerveModuleControl SwerveModule::GetControl(frc::SwerveModulePosition desired) {
    SwerveModuleControl calculatedControl;
    frc::SwerveModulePosition desiredState{};
    desiredState.distance = desired.distance;
    desiredState.angle = desired.angle + frc::Rotation2d{m_configs.kRotationOffset};

    calculatedControl.WithDrivePositionTarget(desiredState.distance).WithDriveFeedback();

    calculatedControl.WithAzimuthPosition(desiredState.angle.Degrees());

    return calculatedControl;
}

void SwerveModule::OpenLoop(SwerveModuleControl& control) {
    switch (control.drive.outputType) {
        case DriveModuleControl::OutputType::kDuty:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::DutyCycleOut{control.drive.open.duty});
            break;
        case DriveModuleControl::OutputType::kVoltage:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::VoltageOut{control.drive.open.voltage});
            break;
        case DriveModuleControl::OutputType::kTorque:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::TorqueCurrentFOC{control.drive.open.torqueCurrent});
            break;
    }
}

void SwerveModule::ClosedLoop(SwerveModuleControl& control, Feedforward ff) {
    switch (control.drive.targetType) {
        case DriveModuleControl::TargetType::kPosition:
            ClosedPositionLoop(control, ff);
            break;
        case DriveModuleControl::TargetType::kVelocity:
            ClosedVelocityLoop(control, ff);
            break;
        case DriveModuleControl::TargetType::kMotionMagicPosition:
            ClosedMagicPositionLoop(control, ff);
            break;
        case DriveModuleControl::TargetType::kMotionMagicVelocity:
            ClosedMagicVelocityLoop(control, Feedforward{0_A, 0_V, units::scalar_t{0}}, control.drive.ff.accel);
            break;
    }
}

void SwerveModule::ClosedPositionLoop(SwerveModuleControl& control, Feedforward ff) {
    switch (control.drive.outputType) {
        case DriveModuleControl::OutputType::kDuty:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::PositionDutyCycle{control.drive.close.position / m_configs.kGearedRotationToOutputDistance}
                                            .WithSlot(0)
                                            .WithFeedForward(ff.duty));
            break;
        case DriveModuleControl::OutputType::kVoltage:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::PositionVoltage{control.drive.close.position / m_configs.kGearedRotationToOutputDistance}.WithSlot(1).WithFeedForward(
                    ff.voltage));
            break;
        case DriveModuleControl::OutputType::kTorque:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::PositionTorqueCurrentFOC{control.drive.close.position / m_configs.kGearedRotationToOutputDistance}
                    .WithSlot(2)
                    .WithFeedForward(ff.current));
            break;
    }
}

void SwerveModule::ClosedVelocityLoop(SwerveModuleControl& control, Feedforward ff) {
    switch (control.drive.outputType) {
        case DriveModuleControl::OutputType::kDuty:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{control.drive.close.velocity / m_configs.kGearedRotationToOutputDistance}
                                            .WithSlot(0)
                                            .WithFeedForward(ff.duty));
            break;
        case DriveModuleControl::OutputType::kVoltage:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::VelocityVoltage{control.drive.close.velocity / m_configs.kGearedRotationToOutputDistance}.WithSlot(1).WithFeedForward(
                    ff.voltage));
            break;
        case DriveModuleControl::OutputType::kTorque:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::VelocityTorqueCurrentFOC{control.drive.close.velocity / m_configs.kGearedRotationToOutputDistance}
                    .WithSlot(2)
                    .WithFeedForward(ff.current));
            break;
    }
}

void SwerveModule::ClosedMagicPositionLoop(SwerveModuleControl& control, Feedforward ff) {
    switch (control.drive.outputType) {
        case DriveModuleControl::OutputType::kDuty:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::MotionMagicDutyCycle{control.drive.close.position / m_configs.kGearedRotationToOutputDistance}
                                            .WithSlot(0)
                                            .WithFeedForward(ff.duty));
            break;
        case DriveModuleControl::OutputType::kVoltage:
            m_drivingTalonFx.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{control.drive.close.position / m_configs.kGearedRotationToOutputDistance}
                                            .WithSlot(1)
                                            .WithFeedForward(ff.voltage));
            break;
        case DriveModuleControl::OutputType::kTorque:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC{control.drive.close.position / m_configs.kGearedRotationToOutputDistance}
                    .WithSlot(2)
                    .WithFeedForward(ff.current));
            break;
    }
}

void SwerveModule::ClosedMagicVelocityLoop(SwerveModuleControl& control, Feedforward ff, units::meters_per_second_squared_t accel) {
    switch (control.drive.outputType) {
        case DriveModuleControl::OutputType::kDuty:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::MotionMagicVelocityDutyCycle{control.drive.close.velocity / m_configs.kGearedRotationToOutputDistance}
                    .WithSlot(0)
                    .WithFeedForward(ff.duty)
                    .WithAcceleration(accel / m_configs.kGearedRotationToOutputDistance));
            break;
        case DriveModuleControl::OutputType::kVoltage:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::MotionMagicVelocityVoltage{control.drive.close.velocity / m_configs.kGearedRotationToOutputDistance}
                    .WithSlot(1)
                    .WithFeedForward(ff.voltage)
                    .WithAcceleration(accel / m_configs.kGearedRotationToOutputDistance));
            break;
        case DriveModuleControl::OutputType::kTorque:
            m_drivingTalonFx.SetControl(
                ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC{control.drive.close.velocity / m_configs.kGearedRotationToOutputDistance}
                    .WithSlot(2)
                    .WithFeedForward(ff.current)
                    .WithAcceleration(accel / m_configs.kGearedRotationToOutputDistance));
            break;
    }
}

SwerveModule::Feedforward SwerveModule::CalculateFeedforward(SwerveModuleControl& control) {
    switch (control.drive.ffOrigin) {
        case DriveModuleControl::FeedforwardOrigin::kAxis:
            return CalculateFeedforward(control.drive.ff.robotRelativeX, control.drive.ff.robotRelativeY);
        case DriveModuleControl::FeedforwardOrigin::kLinear:
            return CalculateFeedforward(control.drive.ff.linearForces);
        case DriveModuleControl::FeedforwardOrigin::kAcceleration: {
            switch (control.drive.outputType) {
                case DriveModuleControl::OutputType::kDuty:
                    return CalculateFeedforwardDuty(control.drive.ff.accel);
                case DriveModuleControl::OutputType::kVoltage:
                    return CalculateFeedforwardVoltage(control.drive.ff.accel);
                case DriveModuleControl::OutputType::kTorque:
                    return CalculateFeedforwardCurrent(control.drive.ff.accel);
            }
        }
    }

    return Feedforward{0_A, 0_V, units::scalar_t{0}};
}

SwerveModule::Feedforward SwerveModule::CalculateFeedforward(units::newton_t x, units::newton_t y) {
    Feedforward calculated;
    units::newton_t distance = units::newton_t{hypot(x.value(), y.value())};

    frc::Rotation2d feedforwardRotation{x.value(), y.value()};
    frc::Rotation2d moduleFeedforwardRototation =
        feedforwardRotation.RotateBy(-frc::Rotation2d{units::turn_t{m_azimuthAbsoluteEncoder.GetPosition()} + m_configs.kRotationOffset});

    units::newton_meter_t torque =
        (distance * moduleFeedforwardRototation.Cos()) * ((m_configs.kWheelCircumference / (2 * std::numbers::pi)) / m_configs.kDriveGearRatio);

    calculated.current = torque / m_drivingTalonFx.GetMotorKT(true).GetValue();
    calculated.voltage = (12_V / m_drivingTalonFx.GetMotorStallCurrent(true).GetValue()) * calculated.current;
    calculated.duty = calculated.voltage / 12_V;

    return calculated;
}

SwerveModule::Feedforward SwerveModule::CalculateFeedforward(units::newton_t linear) {
    Feedforward calculated;

    units::newton_meter_t torque = linear * ((m_configs.kWheelCircumference / (2 * std::numbers::pi)) / m_configs.kDriveGearRatio);

    calculated.current = torque / m_drivingTalonFx.GetMotorKT(true).GetValue();
    calculated.voltage = (12_V / m_drivingTalonFx.GetMotorStallCurrent(true).GetValue()) * calculated.current;
    calculated.duty = calculated.voltage / 12_V;

    return calculated;
}

SwerveModule::Feedforward SwerveModule::CalculateFeedforwardDuty(units::meters_per_second_squared_t accel) {
    Feedforward calculated;

    calculated.duty = m_configs.kDriveDutyGain.sva.kA * accel;
    calculated.voltage = calculated.duty * 12_V;
    calculated.current = calculated.duty * m_drivingTalonFx.GetMotorStallCurrent(true).GetValue();

    return calculated;
}

SwerveModule::Feedforward SwerveModule::CalculateFeedforwardVoltage(units::meters_per_second_squared_t accel) {
    Feedforward calculated;

    calculated.voltage = m_configs.kDriveVoltageGain.sva.kA * accel;
    calculated.duty = calculated.voltage / 12_V;
    calculated.current = (calculated.duty) * m_drivingTalonFx.GetMotorStallCurrent(true).GetValue();

    return calculated;
}

SwerveModule::Feedforward SwerveModule::CalculateFeedforwardCurrent(units::meters_per_second_squared_t accel) {
    Feedforward calculated;

    calculated.current = m_configs.kDriveTorqueGain.sva.kA * accel;
    calculated.voltage = (calculated.current / m_drivingTalonFx.GetMotorStallCurrent(true).GetValue()) * 12_V;
    calculated.duty = calculated.voltage / 12_V;

    return calculated;
}
