#include "commands/SwerveJoystickMoveCommand.hpp"
#include "Constants.hpp"
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Timer.h>
#include <units/math.h>
#include <numbers>

#include "swerve/SwerveRequest.hpp"

SwerveJoystickMoveCommand::SwerveJoystickMoveCommand(SwerveSubsystem* subsystem, frc2::CommandJoystick& joystick)
    : m_subsystem{subsystem},
      m_joystick{joystick.GetHID()},
      m_limiterX{Swerve::TeleopOperator::kTransLimiter},
      m_limiterY{Swerve::TeleopOperator::kTransLimiter},
      m_limiterMag{Swerve::TeleopOperator::kMagLimiter},
      m_limiterA{Swerve::TeleopOperator::kAngleLimiter} {
    AddRequirements(m_subsystem);
    SetName("Swerve Universal Joystick Command");
}

void SwerveJoystickMoveCommand::Initialize() {
    // Ensure limiters start at current joystick state to prevent "jumps"
    m_limiterX.Reset(units::scalar_t{0.0});
    m_limiterY.Reset(units::scalar_t{0.0});
    m_limiterMag.Reset(units::scalar_t{0.0});
    m_limiterA.Reset(units::scalar_t{0.0});
    m_prevTime = frc::Timer::GetTimestamp();
    m_currentTranslationDir = frc::Rotation2d{};
    m_currentTranslationMag = units::dimensionless::scalar_t{0.0};
}


/**
 * Steps an angle toward a target using the shortest path (circular interpolation).
 */
static units::radian_t StepTowardsCircular(units::radian_t current, units::radian_t target, units::radian_t stepsize) {
    // Use Rotation2d to find the shortest angular distance
    // (target - current) result is normalized to [-Pi, Pi]
    units::radian_t error = (frc::Rotation2d{target} - frc::Rotation2d{current}).Radians();

    if (units::math::abs(error) <= stepsize) {
        return target;
    }

    // Use copysign to step in the direction of the shortest path
    return current + units::math::copysign(stepsize, error);
}

/**
 * Finds the magnitude of the smallest distance between two angles.
 */
static units::radian_t AngleDifference(units::radian_t angleA, units::radian_t angleB) {
    return units::math::abs(frc::AngleModulus(angleA - angleB));
}

void SwerveJoystickMoveCommand::Execute() {
    // 1. Inputs & Dashboard
    double throttle = (-m_joystick.GetRawAxis(3) + 1) / 2.0;
    frc::SmartDashboard::PutNumber("Throttle", throttle);

    // Toggle logic
    if (m_joystick.GetRawButtonPressed(2)) m_fieldCentric = !m_fieldCentric;
    if (m_joystick.GetRawButtonPressed(5)) m_subsystem->SeedFieldCentric();
    if (m_joystick.GetRawButtonPressed(6)) m_axisLimited = !m_axisLimited;
    frc::SmartDashboard::PutBoolean("Field Centric", m_fieldCentric);
    frc::SmartDashboard::PutBoolean("Axis Limited", m_axisLimited);

    // Brake logic (Early exit)
    if (m_joystick.GetRawButton(1)) {
        m_subsystem->SetControl(SwerveDriveBrake{});
        return;
    }

    double outX = 0.0;
    double outY = 0.0;

    // 2. Control Logic Branching
    if (m_axisLimited) {
        // Linear Axis Logic: Apply deadband to raw inputs, then smooth with limiters
        double deadX = frc::ApplyDeadband(m_joystick.GetRawAxis(1), deadbandTran);
        double deadY = frc::ApplyDeadband(m_joystick.GetRawAxis(0), deadbandTran);

        outX = m_limiterX.Calculate(deadX);
        outY = m_limiterY.Calculate(deadY);
    } else {
        // Polar Logic: Convert to Mag/Dir, apply deadband to Mag, then smooth

        // Left is negative, right is positive (correct for atan2)
        units::dimensionless::scalar_t rawX{m_joystick.GetRawAxis(1)};
        // Forward is negative, back is positive (reverse for atan2)
        units::dimensionless::scalar_t rawY{-m_joystick.GetRawAxis(0)};

        units::dimensionless::scalar_t mag = units::math::hypot(rawX, rawY);
        units::radian_t dir = units::math::atan2(rawY, rawX);  // Logic preserved from your snippet

        units::dimensionless::scalar_t deadMag = frc::ApplyDeadband(mag, units::dimensionless::scalar_t{deadbandMag});


        units::unit_t<units::compound_unit<units::radians, units::inverse<units::second>>> directionSlewRate{
            (m_currentTranslationMag > 1e-4) ? units::math::abs(Swerve::TeleopOperator::kDirLimiter / m_currentTranslationMag) : 500.0_rad / 1_s};

        units::second_t currentTime = frc::Timer::GetTimestamp();
        units::second_t elapsedTime = currentTime - m_prevTime;
        units::radian_t angleDif = AngleDifference(dir, m_currentTranslationDir.Radians());

        if (angleDif < (0.45_rad * std::numbers::pi)) {
            m_currentTranslationDir = frc::Rotation2d{StepTowardsCircular(m_currentTranslationDir.Radians(), dir, directionSlewRate * elapsedTime)};
            m_currentTranslationMag = m_limiterMag.Calculate(deadMag);
        } else if (angleDif > (0.85_rad * std::numbers::pi)) {
            if (m_currentTranslationMag > 1e-4) {  // some small number to avoid floating-point errors with
                                                   // equality checking
                // keep currentTranslationDir unchanged
                m_currentTranslationMag = m_limiterMag.Calculate(0.0);
            } else {
                m_currentTranslationDir = m_currentTranslationDir + frc::Rotation2d{180_deg};
                m_currentTranslationMag = m_limiterMag.Calculate(deadMag);
            }
        } else {
            m_currentTranslationDir = StepTowardsCircular(m_currentTranslationDir.Radians(), dir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_limiterMag.Calculate(0.0);
        }

        m_prevTime = currentTime;

        // Need to reverse motion because X,Y in this coordinate system are flipped
        outX = units::math::cos(m_currentTranslationDir.Radians()) * m_currentTranslationMag;
        outY = units::math::sin(m_currentTranslationDir.Radians()) * m_currentTranslationMag;
    }

    double deadRot = frc::ApplyDeadband(m_joystick.GetRawAxis(2), deadbandA);
    double outRot = -m_limiterA.Calculate(deadRot);

    // 4. Output to Subsystem
    auto vx = outX * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;
    auto vy = outY * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;
    auto va = outRot * Swerve::TeleopOperator::kDriveAngleSpeedMax;

    if (m_fieldCentric) {
        m_subsystem->SetControl(FieldCentric{}
                                    .WithVelocityX(vx)
                                    .WithVelocityY(vy)
                                    .WithRotationalRate(va)
                                    .WithDriveRequestType(ModuleDriveRequestType::Velocity)
                                    .WithSteerRequestType(ModuleSteerRequestType::Position)
                                    .WithForwardPerspective(ForwardPerspectiveValue::OperatorPerspective));
    } else {
        m_subsystem->SetControl(RobotCentric{}
                                    .WithVelocityX(vx)
                                    .WithVelocityY(vy)
                                    .WithRotationalRate(va)
                                    .WithDriveRequestType(ModuleDriveRequestType::Velocity)
                                    .WithSteerRequestType(ModuleSteerRequestType::Position));
    }
}

void SwerveJoystickMoveCommand::End(bool interrupted) {
    m_subsystem->SetControl(SwerveDriveBrake{});
}

bool SwerveJoystickMoveCommand::IsFinished() {
    return false;
}