package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
import frc.robot.claw.ClawConfig;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.RunElevatorManual;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.RunPivotManual;
import frc.robot.intake.commands.SetPivotState;
import frc.robot.intake.commands.ToggleHood;

public class OperatorGamepad extends Gamepad {
    public OperatorGamepad() {
        super("OperatorController", OperatorConfig.kOperatorPort);
    }

    @Override
    public void setupTeleopButtons() {
        // Elevator commands
        gamepad.aButton.onTrue(ElevatorCommands.setState(ElevatorConfig.ElevatorState.kZeroGoal));
        gamepad.xButton.onTrue(ElevatorCommands.setState(ElevatorConfig.ElevatorState.kMid));
        gamepad.yButton.onTrue(ElevatorCommands.setState(ElevatorConfig.ElevatorState.kHighGoal));
        gamepad.startButton.onTrue(new InstantCommand(() -> RobotContainer.elevator.zero()));

        gamepad.leftBumper.onTrue(ClawCommands.toggleState());

        // Intake commands
        gamepad.rightBumper.onTrue(new ToggleHood(RobotContainer.intakeHood));
        gamepad.Dpad.Up.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kHumanPlayer));
        gamepad.Dpad.Down.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kDeployed));
        gamepad.Dpad.Left.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kStowed));
        gamepad.Dpad.Right.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kScoreLow));
        gamepad.leftTriggerButton.whileTrue(new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kOuttake));

        // Manual overrides
        shift().whileTrue(new RunElevatorManual(RobotContainer.elevator, () -> gamepad.rightStick.getY()));
        shift().whileTrue(new RunPivotManual(RobotContainer.intakePivot, () -> -gamepad.leftStick.getX()));
    }

    @Override
    public void setupDisabledButtons() {
    }

    @Override
    public void setupTestButtons() {
    }

    private Trigger shift() {
        return gamepad.rightTriggerButton;
    }
}
