#include "commands/SwerveJoystickLookCommand.hpp"

#include "Constants.hpp"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/Smartdashboard.h>

#include <frc2/command/RunCommand.h>
#include <frc/DriverStation.h>

#include "swerve/SwerveRequest.hpp"

SwerveJoystickLookCommand::SwerveJoystickLookCommand(SwerveSubsystem* const subsystem, frc2::CommandJoystick& joystick)
    : m_subsystem{subsystem}, m_joystick{joystick.GetHID()}, m_limiterMag{Swerve::TeleopOperator::kMagLimiter} {
    AddRequirements(m_subsystem);
    SetName("Swerve Joystick Look Command");
}

void SwerveJoystickLookCommand::Initialize() {
    m_lastRotation = frc::Rotation2d{};
    m_limiterMag.Reset(units::scalar_t{0.0});
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

void SwerveJoystickLookCommand::Execute() {
    if (m_joystick.GetRawButtonPressed(5)) {
        m_subsystem->SeedFieldCentric();
    }
    if (m_joystick.GetRawButtonPressed(4)) {
        m_globalLook = !m_globalLook;
    }
    if (m_joystick.GetRawButton(1)) {
        m_subsystem->SetControl(SwerveDriveBrake{});
        return;
    }
    frc::SmartDashboard::PutBoolean("Field Centric", true);
    frc::Rotation2d desiredRotation;

    units::meters_per_second_t vx = 0_mps;
    units::meters_per_second_t vy = 0_mps;

    units::dimensionless::scalar_t rawX{m_joystick.GetRawAxis(1)};
    // Forward is negative, back is positive (reverse for atan2)
    units::dimensionless::scalar_t rawY{-m_joystick.GetRawAxis(0)};

    units::dimensionless::scalar_t mag = units::math::hypot(rawX, rawY);
    units::radian_t dir = units::math::atan2(rawX, rawY);

    if (m_globalLook) {
        frc::SmartDashboard::PutBoolean("Axis Limited", false);
        double throttle = (-m_joystick.GetRawAxis(3) + 1) / 2.0;
        frc::SmartDashboard::PutNumber("Throttle", throttle);

        units::dimensionless::scalar_t deadMag = frc::ApplyDeadband(mag, units::dimensionless::scalar_t{Swerve::TeleopOperator::kDriveDeadband});

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
        vx = units::math::cos(m_currentTranslationDir.Radians()) * m_currentTranslationMag * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;
        vy = -units::math::sin(m_currentTranslationDir.Radians()) * m_currentTranslationMag * throttle * Swerve::TeleopOperator::kDriveMoveSpeedMax;

        frc::Pose3d pose = m_subsystem->GetState().Pose.TransformBy(Shooter::kFlywheel.Location);

        frc::Translation3d target = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kBlue ?
                                        Shooter::kBlueShootLocation :
                                        Shooter::kRedShootLocation;

        desiredRotation = (target.ToTranslation2d() - pose.ToPose2d().Translation()).Angle();
    } else {
        if (mag() <= Swerve::TeleopOperator::kDriveDeadband) {
            desiredRotation = m_lastRotation;
        } else {
            desiredRotation = frc::Rotation2d{dir};
        }
    }

    m_lastRotation = desiredRotation;
    m_subsystem->SetControl(FieldCentricFacingAngle{}
                                .WithHeadingPID(0.05, 0, 0.001)
                                .WithForwardPerspective(ForwardPerspectiveValue::OperatorPerspective)
                                .WithTargetDirection(desiredRotation)
                                .WithVelocityX(vx)
                                .WithVelocityY(vy)
                                .WithDriveRequestType(ModuleDriveRequestType::Velocity)
                                .WithSteerRequestType(ModuleSteerRequestType::Position)
                                .WithMaxAbsRotationalRate(Swerve::Auto::kMaxAngularSpeed * 0.1));
}

void SwerveJoystickLookCommand::End(bool interrupted) {
    m_subsystem->SetControl(SwerveDriveBrake{});
}

bool SwerveJoystickLookCommand::IsFinished() {
    return false;
}