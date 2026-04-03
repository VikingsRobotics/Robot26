// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include <vector>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/CageSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"
#include "subsystems/SwerveSubsystem.hpp"
#include "subsystems/ShooterSubsystem.hpp"

#include "commands/SwerveJoystickMoveCommand.hpp"
#include "commands/SwerveJoystickLookCommand.hpp"
#include "commands/ShooterControllerCommand.hpp"

#include "commands/SwerveImportantCommand.hpp"

class RobotContainer {
public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();

public:
    void ConfigureBindings();
    CageSubsystem cageSubsystem{};
    ElevatorSubsystem elevatorSubsystem{};
    SwerveSubsystem swerveSubsystem{};
    ShooterSubsystem shooterSubsystem{};

    frc::SendableChooser<frc2::Command*> autoChooser{};

    frc2::CommandJoystick joystick;
    frc2::CommandXboxController controller;

    SwerveJoystickMoveCommand moveSwerve{&swerveSubsystem, joystick};
    SwerveJoystickLookCommand lookSwerve{&swerveSubsystem, joystick};

    ShooterControllerCommand controllerShooter{&shooterSubsystem, controller};

    SwerveImportantCommand importantCommand{&swerveSubsystem};
};
