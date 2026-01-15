// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"

#include "commands/SwerveAlignAprilTagCommand.h"
#include "commands/SwerveControllerCommand.h"
#include "commands/SwerveJoystickCommand.h"
#include "commands/SwerveSysIdRoutine.h"

#include <frc/RobotState.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/CommandUtil.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() : joystick{Drive::ControllerPorts::kDriverControllerPort}, xboxController{Elevator::ControllerPorts::kDriverControllerPort} {
#ifndef NO_SWERVE
    swerveSubsystem.SetDefaultCommand(SwerveJoystickCommand{&swerveSubsystem, joystick});
    joystick.Button(6).OnTrue(swerveSubsystem.GetDefaultCommand());
#ifndef NO_VISION
    joystick.Button(3)
        .Debounce(Drive::TeleopOperator::kVisionDebounce)
        .OnTrue(SwerveAlignAprilTagCommand{&swerveSubsystem, &visionProvider, Drive::Align::Location::kLeftBranch, Drive::Align::Location::kBranchThreshold,
                                           Drive::Align::System::kTranslationP, 0, 0, Drive::Align::System::kRotationP, 0, 0}
                    .ToPtr());
    joystick.Button(4)
        .Debounce(Drive::TeleopOperator::kVisionDebounce)
        .OnTrue(SwerveAlignAprilTagCommand{&swerveSubsystem, &visionProvider, Drive::Align::Location::kRightBranch, Drive::Align::Location::kBranchThreshold,
                                           Drive::Align::System::kTranslationP, 0, 0, Drive::Align::System::kRotationP, 0, 0}
                    .ToPtr());
#endif
#endif
#ifndef NO_ELEVATOR
    elevatorSubsystem.SetDefaultCommand(
        frc2::cmd::Idle(frc2::Requirements{&elevatorSubsystem}).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    repellerSubsystem.SetDefaultCommand(
        frc2::cmd::Idle(frc2::Requirements{&repellerSubsystem}).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
#endif
#ifndef NO_ARM
    armSubsystem.SetDefaultCommand(frc2::cmd::Idle(frc2::Requirements{&armSubsystem}).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
#endif
#ifndef NO_ROLLER
    rollerSubsystem.SetDefaultCommand(
        frc2::cmd::Idle(frc2::Requirements{&rollerSubsystem}).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
#endif

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    // PrintDisabledSystems();
#ifndef NO_ELEVATOR_HEIGHT_COMMAND
    BindElevatorCommand();
#endif
#ifndef NO_ARM_ROTATION_COMMAND
    BindArmCommand();
#endif
#ifndef NO_ROLLER
    BindRollerCommands();
#endif
#ifndef NO_SWERVE_SYSID_COMMAND
    ConfigureSwerveSysId();
#endif
#ifndef NO_SWERVE
    BindCoralCommands();
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData(&autoChooser);
#endif
}

#ifndef NO_SWERVE_SYSID_COMMAND

void RobotContainer::ConfigureSwerveSysId() {
    swerveSysIdTestMoveOnly.Set(false);

    SwerveSysIdRoutine commandHolder{&swerveSubsystem};

    /* Added all the translation SysId commands*/

    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineTranslation.Quasistatic(frc2::sysid::kForward)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Translation Quasistatic Forward"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineTranslation.Quasistatic(frc2::sysid::kReverse)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Translation Quasistatic Reverse"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineTranslation.Dynamic(frc2::sysid::kForward)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Translation Dyanamic Forward"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineTranslation.Dynamic(frc2::sysid::kReverse)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Translation Dyanamic Reverse"));

    /* Add all the rotation SysId Commands */

    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineRotation.Quasistatic(frc2::sysid::kForward)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Rotation Quasistatic Forward"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineRotation.Quasistatic(frc2::sysid::kReverse)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Rotation Quasistatic Reverse"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineRotation.Dynamic(frc2::sysid::kForward)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Rotation Dynamic Forward"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineRotation.Dynamic(frc2::sysid::kReverse)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Rotation Dynamic Reverse"));

    /* Adds all steer SysId commands */

    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineSteer.Quasistatic(frc2::sysid::kForward)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Steer Quasistatic Forward"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineSteer.Quasistatic(frc2::sysid::kReverse)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Steer Quasistatic Reverse"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineSteer.Dynamic(frc2::sysid::kForward)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Steer Dynamic Forward"));
    SwerveSysId.emplace_back(
        frc2::cmd::Either(
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(true); }).AndThen(commandHolder.sysIdRoutineSteer.Dynamic(frc2::sysid::kReverse)),
            frc2::cmd::RunOnce([&]() { swerveSysIdTestMoveOnly.Set(false); }), []() -> bool { return frc::RobotState::IsTest(); })
            .WithName("Steer Dynamic Reverse"));

    /* Publish Commands to Test Shuffleboard */

    // Create them before using them (so hopefully it works correctly)
    frc::Shuffleboard::GetTab("Test").GetLayout("Translation", frc::BuiltInLayouts::kList);
    frc::Shuffleboard::GetTab("Test").GetLayout("Rotation", frc::BuiltInLayouts::kList);
    frc::Shuffleboard::GetTab("Test").GetLayout("Steer", frc::BuiltInLayouts::kList);

    // Publish them to the lists
    for (size_t index = 0; index < SwerveSysId.size(); ++index) {
        switch ((index / 4)) {
            case 0:
                frc::Shuffleboard::GetTab("Test")
                    .GetLayout("Translation", frc::BuiltInLayouts::kList)
                    .Add(*SwerveSysId.at(index).get())
                    .WithWidget(frc::BuiltInWidgets::kCommand);
                break;
            case 1:
                frc::Shuffleboard::GetTab("Test")
                    .GetLayout("Rotation", frc::BuiltInLayouts::kList)
                    .Add(*SwerveSysId.at(index).get())
                    .WithWidget(frc::BuiltInWidgets::kCommand);
                break;
            case 2:
                frc::Shuffleboard::GetTab("Test")
                    .GetLayout("Steer", frc::BuiltInLayouts::kList)
                    .Add(*SwerveSysId.at(index).get())
                    .WithWidget(frc::BuiltInWidgets::kCommand);
            default:
                break;
        }
    }

    frc::Shuffleboard::GetTab("Test").Add("Default (Stop)", *swerveSubsystem.GetDefaultCommand()).WithWidget(frc::BuiltInWidgets::kCommand);
}

#endif
#ifndef NO_ELEVATOR

void RobotContainer::BindElevatorCommand() {
    xboxController.A()
        .Debounce(Elevator::TeleopOperator::kDebounce)
        .OnTrue(HeightCommand{&elevatorSubsystem, Elevator::Destination::kCollectionHeight, Elevator::Destination::kAllowableSwitchTime}.ToPtr());
    xboxController.B()
        .Debounce(Elevator::TeleopOperator::kDebounce)
        .OnTrue(HeightCommand{&elevatorSubsystem, Elevator::Destination::kSecondGoal, Elevator::Destination::kAllowableSwitchTime}.ToPtr());
    xboxController.X()
        .Debounce(Elevator::TeleopOperator::kDebounce)
        .OnTrue(HeightCommand{&elevatorSubsystem, Elevator::Destination::kThirdGoal, Elevator::Destination::kAllowableSwitchTime}.ToPtr());
    xboxController.Y()
        .Debounce(Elevator::TeleopOperator::kDebounce)
        .OnTrue(HeightCommand{&elevatorSubsystem, Elevator::Destination::kFourthGoal, Elevator::Destination::kAllowableSwitchTime}.ToPtr());
    xboxController.RightBumper()
        .Debounce(Elevator::TeleopOperator::kDebounce)
        .OnTrue(HeightCommand{&elevatorSubsystem, Elevator::Destination::kAbsorbCoralHeight, Elevator::Destination::kAllowableSwitchTime}.ToPtr());

    xboxController.LeftBumper()
        .Debounce(Elevator::TeleopOperator::kDebounce)
        .OnTrue(frc2::cmd::RunOnce(
                    [&]() -> void {
                        double current = repellerSubsystem.GetRepellerWheelSpeed();
                        repellerSubsystem.SetRepellerWheel(current == 0.0 ? 1.0 : 0.0);
                    },
                    {&repellerSubsystem})
                    .WithName("Repeller Turns"));
}

#endif
#ifndef NO_ARM

void RobotContainer::BindArmCommand() {
    xboxController.Start()
        .Debounce(Arm::TeleopOperator::kDebounce)
        .OnTrue(RotationCommand{&armSubsystem, Arm::Destination::kTopTurn, Arm::Destination::kAllowableSwitchTime}.ToPtr());

    /*	xboxController.POVLeft().Debounce(Arm::TeleopOperator::kDebounce).OnTrue(
     RotationCommand { &armSubsystem, Arm::Destination::kMiddleTurn,
     Arm::Destination::kAllowableSwitchTime }.ToPtr()); */

    /*	xboxController.POVRight().Debounce(Arm::TeleopOperator::kDebounce).OnTrue(
     RotationCommand { &armSubsystem, Arm::Destination::kMinTurn,
     Arm::Destination::kAllowableSwitchTime }.ToPtr()); */

    xboxController.Back()
        .Debounce(Arm::TeleopOperator::kDebounce)
        .OnTrue(RotationCommand{&armSubsystem, Arm::Destination::kBottomTurn, Arm::Destination::kAllowableSwitchTime}.ToPtr());
}

#endif
#ifndef NO_ROLLER
void RobotContainer::BindRollerCommands() {
    using namespace frc2::cmd;
    xboxController.LeftTrigger(Roller::TeleopOperator::kDriveDeadband)
        .Debounce(Roller::TeleopOperator::kDebounce)
        .ToggleOnTrue(RunOnce([&]() -> void { rollerSubsystem.PutRollerDown(); }, {&rollerSubsystem}).WithName("Down"));
    xboxController.LeftTrigger(Roller::TeleopOperator::kDriveDeadband)
        .Debounce(Roller::TeleopOperator::kDebounce)
        .ToggleOnFalse(RunOnce([&]() -> void { rollerSubsystem.PutRollerUp(); }, {&rollerSubsystem}).WithName("Up"));

    xboxController.LeftStick()
        .Debounce(Roller::TeleopOperator::kDebounce)
        .OnTrue(RunOnce([&]() -> void { rollerSubsystem.SetRollerWheel(0.6); }, {&rollerSubsystem}).WithName("Forward"));
    xboxController.RightStick()
        .Debounce(Roller::TeleopOperator::kDebounce)
        .OnTrue(RunOnce([&]() -> void { rollerSubsystem.SetRollerWheel(-0.6); }, {&rollerSubsystem}).WithName("Reverse"));
    xboxController.LeftStick().OnFalse(RunOnce([&]() -> void { rollerSubsystem.SetRollerWheel(0.0); }, {&rollerSubsystem}).OnlyIf([&]() -> bool {
        return !xboxController.RightStick().Get();
    }));
    xboxController.RightStick().OnFalse(RunOnce([&]() -> void { rollerSubsystem.SetRollerWheel(0.0); }, {&rollerSubsystem}).OnlyIf([&]() -> bool {
        return !xboxController.LeftStick().Get();
    }));
}
#endif

#if !defined(NO_ELEVATOR) && !defined(NO_ARM)
void RobotContainer::BindCoralCommands() {
    using namespace frc2::cmd;
    using namespace pathplanner;

    auto atHeight = [&](units::inch_t height) -> bool { return elevatorSubsystem.GetDistance() >= height; };
    auto meetsClearance = [&]() -> bool { return atHeight(Arm::Mechanism::kRequiredClearance); };

    auto meetAngle = [&](units::turn_t endpoint, units::turn_t tolerance) -> bool {
        return units::math::abs(armSubsystem.GetRotation() - endpoint) < tolerance;
    };

    NamedCommands::registerCommand(
        "Absorb Coral",
        RotationCommand{&armSubsystem, Arm::Destination::kCollectTurn, Arm::Destination::kAllowableSwitchTime, Arm::Destination::kAllowableErrorCommand}
            .ToPtr()
            .AlongWith(RunOnce([&]() -> void { repellerSubsystem.SetRepellerWheel(0.0); }, {&repellerSubsystem}))
            .AndThen(HeightCommand{&elevatorSubsystem, Elevator::Destination::kAbsorbCoralHeight, Elevator::Destination::kAllowableSwitchTime,
                                   Elevator::Destination::kAllowableErrorCommand}
                         .ToPtr())
            .WithName("Absorbing Coral"));
    NamedCommands::registerCommand(
        "Low Rung", HeightCommand{&elevatorSubsystem, Elevator::Destination::kSecondGoal, Elevator::Destination::kAllowableSwitchTime,
                                  Elevator::Destination::kAllowableErrorCommand}
                        .ToPtr()
                        .AlongWith(Race(WaitUntil(meetsClearance), Wait(2_s))
                                       .AndThen(RotationCommand{&armSubsystem, Arm::Destination::kTopTurn, Arm::Destination::kAllowableSwitchTime,
                                                                Arm::Destination::kAllowableErrorCommand}
                                                    .ToPtr()))
                        .WithName("Push Second Goal"));
    NamedCommands::registerCommand(
        "Middle Rung", HeightCommand{&elevatorSubsystem, Elevator::Destination::kThirdGoal, Elevator::Destination::kAllowableSwitchTime,
                                     Elevator::Destination::kAllowableErrorCommand}
                           .ToPtr()
                           .AlongWith(Race(WaitUntil(meetsClearance), Wait(2_s))
                                          .AndThen(RotationCommand{&armSubsystem, Arm::Destination::kTopTurn, Arm::Destination::kAllowableSwitchTime,
                                                                   Arm::Destination::kAllowableErrorCommand}
                                                       .ToPtr()))
                           .WithName("Push Third Goal"));
    NamedCommands::registerCommand(
        "High Rung", HeightCommand{&elevatorSubsystem, Elevator::Destination::kThirdGoal, Elevator::Destination::kAllowableSwitchTime,
                                   Elevator::Destination::kAllowableErrorCommand}
                         .ToPtr()
                         .AlongWith(Race(WaitUntil(meetsClearance), Wait(2_s))
                                        .AndThen(RotationCommand{&armSubsystem, Arm::Destination::kTopTurn, Arm::Destination::kAllowableSwitchTime,
                                                                 Arm::Destination::kAllowableErrorCommand}
                                                     .ToPtr()))
                         .AndThen(WaitUntil([meetAngle]() -> bool { return meetAngle(Arm::Destination::kTopTurn, Arm::Destination::kAllowableErrorCommand); }))
                         .AndThen(HeightCommand{&elevatorSubsystem, Elevator::Destination::kFourthGoal, Elevator::Destination::kAllowableSwitchTime,
                                                Elevator::Destination::kAllowableErrorCommand}
                                      .ToPtr())
                         .AndThen(RotationCommand{&armSubsystem, Arm::Destination::kMiddleTurn, Arm::Destination::kAllowableSwitchTime,
                                                  Arm::Destination::kAllowableErrorCommand}
                                      .ToPtr()));
    NamedCommands::registerCommand(
        "Collect Coral",
        HeightCommand{&elevatorSubsystem, Elevator::Destination::kCollectionHeight, Elevator::Destination::kAllowableSwitchTime}.ToPtr().WithName(
            "Collecting Coral"));
    NamedCommands::registerCommand("Start Repel", RunOnce([&]() -> void { repellerSubsystem.SetRepellerWheel(1.0); }, {&repellerSubsystem}));
    NamedCommands::registerCommand("Stop Repel", RunOnce([&]() -> void { repellerSubsystem.SetRepellerWheel(0.0); }, {&repellerSubsystem}));
}
#endif

void RobotContainer::PrintDisabledSystems() {
#ifdef NO_SWERVE
    frc::SmartDashboard::PutBoolean("Disabled Swerve", true);
#else
    frc::SmartDashboard::PutBoolean("Disabled Swerve", false);
#endif
#ifdef NO_ELEVATOR
    frc::SmartDashboard::PutBoolean("Disabled Elevator", true);
#else
    frc::SmartDashboard::PutBoolean("Disabled Elevator", false);
#endif
#ifdef NO_ARM
    frc::SmartDashboard::PutBoolean("Disabled Arm", true);
#else
    frc::SmartDashboard::PutBoolean("Disabled Arm", false);
#endif
#ifdef NO_ROLLER
    frc::SmartDashboard::PutBoolean("Disabled Roller", true);
#else
    frc::SmartDashboard::PutBoolean("Disabled Roller", false);
#endif
#ifdef NO_VISION
    frc::SmartDashboard::PutBoolean("Disabled Vision", true);
#else
    frc::SmartDashboard::PutBoolean("Disabled Vision", false);
#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
#ifdef NO_SWERVE
    return nullptr;
#else
    return autoChooser.GetSelected();
#endif
}
