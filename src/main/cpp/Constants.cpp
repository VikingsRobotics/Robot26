#include "constants/Swerve.hpp"
#include "constants/Robot.hpp"

#include <numbers>

namespace Swerve {

rev::spark::SparkMaxConfig& DeviceProperties::GetSparkMaxConfig() {
    static rev::spark::SparkMaxConfig config{};
    config.absoluteEncoder.PositionConversionFactor(2 * std::numbers::pi);
    config.absoluteEncoder.VelocityConversionFactor((2 * std::numbers::pi) / 60);

    config.absoluteEncoder.Inverted(Swerve::DeviceProperties::kInvertEncoder);

    config.closedLoop.PositionWrappingEnabled(true);
    config.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder);

    // * POSITION CLOSED-LOOP * //
    config.closedLoop.OutputRange(-1, 1, rev::spark::kSlot0);
    config.closedLoop.Pid(Swerve::Characterization::Azimuth::kP(), Swerve::Characterization::Azimuth::kI(), Swerve::Characterization::Azimuth::kD(),
                          rev::spark::kSlot0);
    config.closedLoop.feedForward.sva(Swerve::Characterization::Azimuth::kS(), Swerve::Characterization::Azimuth::kV(), Swerve::Characterization::Azimuth::kA(),
                                      rev::spark::kSlot0);
    // * END CLOSED-LOOP * //

    // * SMART MOTION CLOSED-LOOP * //
    config.closedLoop.MinOutput(0, rev::spark::kSlot1);
    config.closedLoop.MaxOutput(std::numbers::pi * 2, rev::spark::kSlot1);
    config.closedLoop.OutputRange(-1, 1, rev::spark::kSlot1);
    config.closedLoop.Pid(Swerve::Characterization::Azimuth::kP(), Swerve::Characterization::Azimuth::kI(), Swerve::Characterization::Azimuth::kD(),
                          rev::spark::kSlot1);
    config.closedLoop.feedForward.sva(Swerve::Characterization::Azimuth::kS(), Swerve::Characterization::Azimuth::kV(), Swerve::Characterization::Azimuth::kA(),
                                      rev::spark::kSlot1);
    // SMART MOTION CONFIGS
    config.closedLoop.maxMotion.CruiseVelocity(Swerve::Auto::kMaxAngularSpeed.value(), rev::spark::kSlot1);
    config.closedLoop.maxMotion.MaxAcceleration(Swerve::Auto::kMaxAngularAcceleration.value(), rev::spark::kSlot1);
    config.closedLoop.maxMotion.PositionMode(rev::spark::MAXMotionConfig::MAXMotionPositionMode::kMAXMotionTrapezoidal, rev::spark::kSlot1);
    config.closedLoop.maxMotion.AllowedProfileError(0, rev::spark::kSlot1);
    // * END CLOSED-LOOP * //

    config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);

    return config;
}

ctre::phoenix6::configs::TalonFXConfiguration DeviceProperties::GetTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration config;

    config.WithVoltage(ctre::phoenix6::configs::VoltageConfigs{}.WithPeakForwardVoltage(12_V).WithPeakReverseVoltage(-12_V));
    config.WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs{}.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake));
    config.WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                         .WithKP(Swerve::Characterization::Drive::kP)
                         .WithKI(Swerve::Characterization::Drive::kI)
                         .WithKD(Swerve::Characterization::Drive::kD)
                         .WithKS(Swerve::Characterization::Drive::kS())
                         .WithKV(Swerve::Characterization::Drive::kV())
                         .WithKA(Swerve::Characterization::Drive::kA())
                         .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Elevator_Static)
                         .WithStaticFeedforwardSign(ctre::phoenix6::signals::StaticFeedforwardSignValue::UseVelocitySign));

    return config;
}

frc::SwerveDriveKinematics<4> System::kDriveKinematics{frc::Translation2d{+Robot::Mechanism::kWheelBase / 2, +Robot::Mechanism::kTrackWidth / 2},
                                                       frc::Translation2d{+Robot::Mechanism::kWheelBase / 2, -Robot::Mechanism::kTrackWidth / 2},
                                                       frc::Translation2d{-Robot::Mechanism::kWheelBase / 2, +Robot::Mechanism::kTrackWidth / 2},
                                                       frc::Translation2d{-Robot::Mechanism::kWheelBase / 2, -Robot::Mechanism::kTrackWidth / 2}};

}  // namespace Swerve