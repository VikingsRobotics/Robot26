#include "subsystems/SwerveModule.hpp"

#include "constants/Swerve.hpp"
#include "constants/ID.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int drivingCANId, const int turningCANId, const units::radian_t chassisAngularOffest)
    : m_drivingTalonFx{drivingCANId, DeviceIdentifier::kCANBus},
      m_turningSparkMax{turningCANId, Swerve::DeviceProperties::kSparkMotorType},
      m_turningAbsoluteEncoder{m_turningSparkMax.GetAbsoluteEncoder()},
      m_chassisAngularOffest{chassisAngularOffest},
      m_sparkLoopController{m_turningSparkMax.GetClosedLoopController()},
      m_getTalonPosition{m_drivingTalonFx.GetPosition().AsSupplier()},
      m_getTalonVelocity{m_drivingTalonFx.GetVelocity().AsSupplier()} {
    // * SETING UP NEO 550 TURING MOTOR CONFIGURATION * //
    m_turningSparkMax.Configure(Swerve::DeviceProperties::GetSparkMaxConfig(), rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);
    // * FINISHED SETTING UP CONFIGURATION * //

    // * SETING UP TALON DRIVE MOTOR CONFIGURATION * //
    m_drivingTalonFx.GetConfigurator().Apply(Swerve::DeviceProperties::GetTalonFXConfig());
    // * FINISHED SETTING UP CONFIGURATION * //

    ResetEncoder();
    Stop();
}

frc::SwerveModuleState SwerveModule::GetState() {
    return frc::SwerveModuleState{m_getTalonVelocity() * Swerve::Mechanism::kDriveMotorToDistance,
                                  frc::Rotation2d{units::angle::radian_t{m_turningAbsoluteEncoder.GetPosition()} - m_chassisAngularOffest}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return frc::SwerveModulePosition{m_getTalonPosition() * Swerve::Mechanism::kDriveMotorToDistance,
                                     frc::Rotation2d{units::angle::radian_t{m_turningAbsoluteEncoder.GetPosition()} - m_chassisAngularOffest}};
}

void SwerveModule::SetState(frc::SwerveModuleState desiredState) {
    frc::SwerveModuleState optimizedDesiredState{};
    optimizedDesiredState.speed = desiredState.speed;
    optimizedDesiredState.angle = desiredState.angle + frc::Rotation2d{m_chassisAngularOffest};

    optimizedDesiredState.Optimize(frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()}));

    if (std::abs(desiredState.speed.value()) > 0.0001) {
        m_drivingTalonFx.SetControl(
            ctre::phoenix6::controls::VelocityVoltage{optimizedDesiredState.speed / Swerve::Mechanism::kDriveMotorToDistance}.WithSlot(0));
    } else {
        m_drivingTalonFx.Set(0);
    }

    if (m_useSmartMotionSparkMax) {
        m_sparkLoopController.SetSetpoint(optimizedDesiredState.angle.Radians().value(), rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                          rev::spark::kSlot1);
    } else {
        m_sparkLoopController.SetSetpoint(optimizedDesiredState.angle.Radians().value(), rev::spark::SparkLowLevel::ControlType::kPosition);
    }
}

void SwerveModule::SetState(frc::SwerveModuleState desiredState, units::newton_t feedforwardX, units::newton_t feedforwardY) {
    frc::SwerveModuleState optimizedDesiredState{};
    optimizedDesiredState.speed = desiredState.speed;
    optimizedDesiredState.angle = desiredState.angle + frc::Rotation2d{m_chassisAngularOffest};

    optimizedDesiredState.Optimize(frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()}));

    Feedforward calculatedFeedforward = CalculateFeedforward(feedforwardX, feedforwardY);

    if (std::abs(desiredState.speed.value()) > 0.0001) {
        m_drivingTalonFx.SetControl(
            ctre::phoenix6::controls::VelocityVoltage{optimizedDesiredState.speed / Swerve::Mechanism::kDriveMotorToDistance}.WithSlot(0).WithFeedForward(
                calculatedFeedforward.voltage));
    } else {
        m_drivingTalonFx.Set(0);
    }

    if (m_useSmartMotionSparkMax) {
        m_sparkLoopController.SetSetpoint(optimizedDesiredState.angle.Radians().value(), rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                          rev::spark::kSlot1);
    } else {
        m_sparkLoopController.SetSetpoint(optimizedDesiredState.angle.Radians().value(), rev::spark::SparkLowLevel::ControlType::kPosition);
    }
}

void SwerveModule::ResetEncoder() {
    m_drivingTalonFx.SetPosition(0_tr);
}

void SwerveModule::Stop() {
    m_drivingTalonFx.Set(0);
    m_turningSparkMax.Set(0);
}

void SwerveModule::Brake() {
    m_drivingTalonFx.Set(0);
    m_drivingTalonFx.SetControl(ctre::phoenix6::controls::StaticBrake{});
    m_turningSparkMax.Set(0);
}

SwerveModule::Feedforward SwerveModule::CalculateFeedforward(units::newton_t feedforwardX, units::newton_t feedforwardY) {
    if ((std::abs(feedforwardX.value()) * std::numeric_limits<UNIT_LIB_DEFAULT_TYPE>::epsilon() <= feedforwardX.value() &&
         feedforwardX.value() >= std::numeric_limits<UNIT_LIB_DEFAULT_TYPE>::epsilon()) ||
        (std::abs(feedforwardY.value()) * std::numeric_limits<UNIT_LIB_DEFAULT_TYPE>::epsilon() <= feedforwardY.value() &&
         feedforwardY.value() >= std::numeric_limits<UNIT_LIB_DEFAULT_TYPE>::epsilon())) {
        Feedforward calculated;

        units::newton_t distance = units::newton_t{hypot(feedforwardX.value(), feedforwardY.value())};

        frc::Rotation2d feedforwardRotation{feedforwardX.value(), feedforwardY.value()};
        frc::Rotation2d moduleFeedforwardRototation =
            feedforwardRotation.RotateBy(frc::Rotation2d{units::radian_t{m_turningAbsoluteEncoder.GetPosition()} + m_chassisAngularOffest});

        calculated.torque = distance * moduleFeedforwardRototation.Cos() * (Swerve::Mechanism::kWheelDiameter / 2) / Swerve::Mechanism::kDriveGearRatio;
        ctre::unit::newton_meters_per_ampere_t motorKT = m_drivingTalonFx.GetMotorKT(true).GetValue();
        calculated.current = calculated.torque / motorKT;
        calculated.voltage = (12_V / m_drivingTalonFx.GetMotorStallCurrent(true).GetValue()) * calculated.current;

        return calculated;
    }
    return Feedforward{units::newton_meter_t{0}, units::ampere_t{0}, units::volt_t{0}};
}
void SwerveModule::GotoRotation(units::radian_t angle) {
    if (m_useSmartMotionSparkMax) {
        m_sparkLoopController.SetSetpoint(angle.value(), rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl, rev::spark::kSlot1);
    } else {
        m_sparkLoopController.SetSetpoint(angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
    }
}