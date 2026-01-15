// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Disable.h"

#include <frc2/command/Command.h>

#include <vector>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/HeightCommand.h"
#include "commands/RotationCommand.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/RepellerSubsystem.h"
#include "subsystems/RollerSubsystem.h"
#include "subsystems/SwerveSubsystem.h"
#include "subsystems/VisionProvider.h"

#include <frc/Alert.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer {
public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();

public:
    void ConfigureBindings();
    void PrintDisabledSystems();
#ifndef NO_SWERVE_SYSID_COMMAND
    void ConfigureSwerveSysId();
    frc::Alert swerveSysIdTestMoveOnly{"Swerve System Identification Command can only be run in Test Mode", frc::Alert::AlertType::kWarning};
    std::vector<frc2::CommandPtr> SwerveSysId{};
#endif
#ifndef NO_SWERVE
    SwerveSubsystem swerveSubsystem{};
    frc::SendableChooser<frc2::Command*> autoChooser{};
#endif
#ifndef NO_VISION
#ifdef NO_SWERVE
    VisionProvider visionProvider{};
#else
    VisionProvider visionProvider{swerveSubsystem};
#endif
#endif
#ifndef NO_ELEVATOR
    void BindElevatorCommand();
    ElevatorSubsystem elevatorSubsystem{};
    RepellerSubsystem repellerSubsystem{};
#endif
#if !defined(NO_ELEVATOR) && !defined(NO_ARM)
    void BindCoralCommands();
#endif
#ifndef NO_ARM
    void BindArmCommand();
    ArmSubsystem armSubsystem{};
#endif
#ifndef NO_ROLLER
    void BindRollerCommands();
    RollerSubsystem rollerSubsystem{};
#endif
    /* [[maybe_unused]] */
    frc2::CommandJoystick joystick;
    /* [[maybe_unused]] */
    frc2::CommandXboxController xboxController;
};
