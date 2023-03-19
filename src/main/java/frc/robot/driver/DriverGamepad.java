package frc.robot.driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.commands.*;

public class DriverGamepad extends Gamepad {
    public DriverGamepad() {
        super("DriverController", DriverConfig.kDriverPort);
    }

    @Override
    public void setupTeleopButtons() {
        // Drivetrain Input
        gamepad.startButton.onTrue(new InstantCommand(() -> RobotContainer.drivetrain.zeroGyro()));

        // Intake input
        gamepad.rightTriggerButton.whileTrue(new RunIntakeManual(RobotContainer.intakeRoller,
                this::getRightTriggerRaw)
        );
        gamepad.leftTriggerButton.whileTrue(
                new RunIntakeManual(RobotContainer.intakeRoller, () -> -getLeftTriggerRaw())
        );

//        shift().whileTrue(new RunPivotManual(RobotContainer.intakePivot, () -> gamepad.rightStick.getX()));
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
}
