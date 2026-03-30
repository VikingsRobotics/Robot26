// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.hpp"

#include "constants.hpp"

#include <frc/RobotState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/CommandUtil.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

#include <frc/DataLogManager.h>

RobotContainer::RobotContainer() : joystick{Swerve::TeleopOperator::kDriverControllerPort}, controller{Shooter::TeleopOperator::kShooterControllerPort} {
    swerveSubsystem.SetDefaultCommand(SwerveJoystickAxisLimitedCommand{&swerveSubsystem, joystick});
    shooterSubsystem.SetDefaultCommand(ShooterControllerCommand{&shooterSubsystem, controller});

    ConfigureBindings();
    frc::DataLogManager::Stop();
}

void RobotContainer::ConfigureBindings() {
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    // frc::SmartDashboard::PutData(&swerveSubsystem);
    // frc::SmartDashboard::PutData(&shooterSubsystem);

    // frc::SmartDashboard::PutData(&autoChooser);

    // frc::SmartDashboard::PutData(&axisSwerve);
    // frc::SmartDashboard::PutData(&polarSwerve);
    // frc::SmartDashboard::PutData(&lookSwerve);

    // frc::SmartDashboard::PutData(&importantCommand);

    swerveSubsystem.AddFunctionCallbackOnTelemerty(
        [this](SwerveDrivetrain::SwerveDriveState const& state) { shooterSubsystem.UpdateShooterPosition(state.Pose); });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}
