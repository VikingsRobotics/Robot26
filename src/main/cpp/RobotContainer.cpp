// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.hpp"

#include "Constants.hpp"

#include <frc/RobotState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/CommandUtil.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/CageControllerCommand.hpp"
#include "commands/ElevatorControllerCommand.hpp"

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

#include <frc/DataLogManager.h>

RobotContainer::RobotContainer() : joystick{Swerve::TeleopOperator::kDriverControllerPort}, controller{Shooter::TeleopOperator::kShooterControllerPort} {
    swerveSubsystem.SetDefaultCommand(SwerveJoystickMoveCommand{&swerveSubsystem, joystick}.WithAxisLimited(false).WithFieldCentric(true).WithDeadband(
        Swerve::TeleopOperator::kDriveDeadband, Swerve::TeleopOperator::kDriveDeadband, Swerve::TeleopOperator::kDriveAngleDeadband));
    shooterSubsystem.SetDefaultCommand(ShooterControllerCommand{&shooterSubsystem, controller});
    cageSubsystem.SetDefaultCommand(CageControllerCommand{&cageSubsystem, controller});
    elevatorSubsystem.SetDefaultCommand(ElevatorControllerCommand{&elevatorSubsystem, controller});

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData(&swerveSubsystem);
    frc::SmartDashboard::PutData(&shooterSubsystem);
    frc::SmartDashboard::PutData(&cageSubsystem);
    frc::SmartDashboard::PutData(&elevatorSubsystem);

    frc::SmartDashboard::PutData(&autoChooser);

    moveSwerve.WithAxisLimited(false).WithFieldCentric(true).WithDeadband(Swerve::TeleopOperator::kDriveDeadband, Swerve::TeleopOperator::kDriveDeadband,
                                                                          Swerve::TeleopOperator::kDriveAngleDeadband);
    frc::SmartDashboard::PutData(&moveSwerve);
    frc::SmartDashboard::PutData(&lookSwerve);

    frc::SmartDashboard::PutData(&importantCommand);

    swerveSubsystem.AddFunctionCallbackOnTelemerty(
        [this](SwerveDrivetrain::SwerveDriveState const& state) { shooterSubsystem.UpdateShooterPosition(state.Pose); });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return nullptr;  // autoChooser.GetSelected();
}
