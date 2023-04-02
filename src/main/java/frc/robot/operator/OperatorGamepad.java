package frc.robot.operator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoBalancing;
import frc.robot.claw.ClawConfig;
import frc.robot.claw.ClawConfig.ClawState;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.commands.ElevatorZero;
import frc.robot.commands.HandoffCone;
import frc.robot.commands.HandoffCube;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreMid;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootHighTeleop;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.RunElevatorManual;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakeConfig.PivotState;
import frc.robot.intake.commands.IntakeCommands;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.RunPivotManual;
import frc.robot.intake.commands.SetPivotState;
import frc.robot.intake.commands.ToggleHood;
import frc.robot.intake.commands.RunIntake.Mode;

public class OperatorGamepad extends Gamepad {
    public OperatorGamepad() {
        super("OperatorController", OperatorConfig.kOperatorPort);
        gamepad.leftStick.setDeadband(0.1);
        gamepad.rightStick.setDeadband(0.1);
    }

    @Override
    public void setupTeleopButtons() {
        // Elevator commands
        gamepad.aButton.onTrue(new ElevatorZero());
        gamepad.xButton.onTrue(new ScoreMid());
    //    gamepad.yButton.onTrue(new ScoreHigh());

//        gamepad.yButton.onTrue(new ScoreHigh());
        gamepad.startButton.onTrue(new InstantCommand(() -> RobotContainer.elevator.zero()));

        gamepad.rightBumper.onTrue(ClawCommands.toggleState());



        gamepad.bButton.onTrue(new HandoffCube());
        gamepad.bButton.and(shift()).onTrue(new HandoffCone());

        // Intake commands
        gamepad.leftBumper.onTrue(new ToggleHood(RobotContainer.intakeHood));
        gamepad.Dpad.Up.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kHumanPlayer));
        gamepad.Dpad.Down.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kDeployed));
        gamepad.Dpad.Left.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kStowed));
        gamepad.Dpad.Right.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kScoreLow));
        // gamepad.startButton.onTrue(new SequentialCommandGroup(new SetPivotState(RobotContainer.intakePivot, PivotState.kShoot),
        //                                 new RunIntake(RobotContainer.intakeRoller, Mode.kOuttake).withTimeout(2)));
        gamepad.startButton.onTrue(new ShootHighTeleop());
        gamepad.leftTriggerButton.whileTrue(new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kShooter));
        // gamepad.leftTriggerButton.whileTrue(IntakeCommands.runRollerManual(RobotContainer.intakeRoller, getLeftTriggerRaw()));

        // Manual overrides
        shift().whileTrue(new RunElevatorManual(RobotContainer.elevator, () -> gamepad.rightStick.getY()));
        shift().whileTrue(new RunPivotManual(RobotContainer.intakePivot, () -> gamepad.leftStick.getY()));
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

    public double getLeftTriggerRaw() {
        return gamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    }
}
