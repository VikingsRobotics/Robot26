// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.hpp"

#include "constants/Swerve.hpp"

#include <frc/RobotState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/CommandUtil.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

#include "commands/SwerveJoystickCommand.hpp"

RobotContainer::RobotContainer() : joystick{Swerve::TeleopOperator::kDriverControllerPort} {
    swerveSubsystem.SetDefaultCommand(SwerveJoystickCommand{&swerveSubsystem, joystick});

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData(&autoChooser);

    frc::SmartDashboard::PutData(&importantCommand);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}
