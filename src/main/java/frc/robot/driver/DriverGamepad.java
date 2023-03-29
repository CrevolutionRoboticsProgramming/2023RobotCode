package frc.robot.driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
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
        gamepad.selectButton.onTrue(new InstantCommand(() -> RobotContainer.drivetrain.resetModules()));

        gamepad.leftTriggerButton.whileTrue(DrivetrainCommands.driveSlowMode(
                this::getDriveTranslationX,
                this::getDriveTranslationY,
                this::getDriveRotation
        ));

        // Intake controls
        gamepad.rightTriggerButton.whileTrue(new ConditionalCommand(
                new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kCube),
                new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kCone),
                () -> getRightTriggerRaw() < 0.9
        ));

        gamepad.rightTriggerButton.whileTrue(DrivetrainCommands.driveIntakeMode(
                this::getDriveTranslationX,
                this::getDriveTranslationY,
                this::getDriveRotation
        ));

        gamepad.rightTriggerButton.onTrue(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kDeployed));
        gamepad.rightTriggerButton.onFalse(new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kStowed));
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
