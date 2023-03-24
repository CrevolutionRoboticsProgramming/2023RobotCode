package frc.robot.driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
import frc.robot.claw.ClawConfig;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.commands.HandoffCone;
import frc.robot.commands.HandoffCube;
import frc.robot.commands.ScoreMid;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.commands.*;

public class DriverGamepad extends Gamepad {
    public DriverGamepad() {
        super("DriverController", DriverConfig.kDriverPort);

        gamepad.leftStick.setDeadband(0.1);
        gamepad.rightStick.setDeadband(0.1);

        gamepad.rightStick.configCurves(2, 1);
    }

    @Override
    public void setupTeleopButtons() {
        gamepad.startButton.onTrue(new InstantCommand(() -> RobotContainer.drivetrain.zeroGyro()));
        gamepad.leftTriggerButton.whileTrue(DrivetrainCommands.driveFieldOrientedSlow(
                this::getDriveTranslationX,
                this::getDriveTranslationY,
                this::getDriveRotation
        ));

        gamepad.rightTriggerButton.whileTrue(new RunIntake(RobotContainer.intakeRoller, () -> {
            if (getRightTriggerRaw() < 0.9) {
                return RunIntake.Mode.kCube;
            } else {
                return RunIntake.Mode.kCone;
            }
        }));

        gamepad.rightBumper.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kDeployed));
    }

    @Override
    public void setupDisabledButtons() {
    }

    @Override
    public void setupTestButtons() {
    }

    public Trigger noShift() {
        return gamepad.leftTriggerButton.negate();
    }

    public Trigger shift() {
        return gamepad.leftTriggerButton;
    }

    public Trigger intakeCube() {
        return gamepad.rightTriggerButton.and(shift());
    }

    public Trigger intakeCone() {
        return gamepad.rightTriggerButton.and(noShift());
    }

    public double getIntakePivotOutput() {
        return gamepad.rightStick.getY();
    }

    public double getRightTriggerRaw() {
        return gamepad.getRawAxis(XboxController.Axis.kRightTrigger.value);
    }

    public double getLeftTriggerRaw() {
        return gamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    }

    public double getDriveTranslationX() {
        return -gamepad.leftStick.getX();
    }

    public double getDriveTranslationY() {
        return gamepad.leftStick.getY();
    }

    public double getDriveRotation() {
        return gamepad.rightStick.getX();
    }
}
