package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.RunElevatorManual;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.commands.RunPivotManual;
import frc.robot.intake.commands.SetPivotState;
import frc.robot.intake.commands.ToggleHood;

public class OperatorGamepad extends Gamepad {
    public OperatorGamepad() {
        super("OperatorController", OperatorConfig.kOperatorPort);
    }

    @Override
    public void setupTeleopButtons() {
        gamepad.xButton.onTrue(ElevatorCommands.elevatorConeLow());
        gamepad.aButton.onTrue(ElevatorCommands.elevatorConeHigh());
        gamepad.bButton.onTrue(ElevatorCommands.elevatorCubeLow());
        gamepad.yButton.onTrue(ElevatorCommands.elevatorCubeHigh());

        gamepad.rightBumper.onTrue(new ToggleHood(RobotContainer.intakeHood));
        gamepad.Dpad.Up.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kHumanPlayer));
        gamepad.Dpad.Down.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kScoreLow));
        gamepad.Dpad.Left.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kStowed));
        gamepad.Dpad.Right.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kDeployed));

        gamepad.leftBumper.onTrue(ClawCommands.toggleClawState());

        gamepad.startButton.onTrue(new InstantCommand(() -> RobotContainer.elevator.resetEncoder()));

        shift().whileTrue(new RunElevatorManual(RobotContainer.elevator, () -> -gamepad.rightStick.getY()));
        shift().whileTrue(new RunPivotManual(RobotContainer.intakePivot, () -> gamepad.leftStick.getX()));
    }

    @Override
    public void setupDisabledButtons() {
    }

    @Override
    public void setupTestButtons() {
    }

    private Trigger shift() {
        return gamepad.leftTriggerButton;
    }
}
