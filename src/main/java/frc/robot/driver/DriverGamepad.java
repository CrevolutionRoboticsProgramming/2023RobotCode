package frc.robot.driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.commands.IntakeCommands;
import frc.robot.intake.commands.RunIntake;

public class DriverGamepad extends Gamepad {
    public DriverGamepad() {
        super("DriverController", DriverConfig.kDriverPort);
    }

    @Override
    public void setupTeleopButtons() {
        gamepad.leftTriggerButton.whileTrue(new RunIntake(RobotContainer.intake, this::getRightTriggerRaw));

        gamepad.aButton.onTrue(new InstantCommand(RobotContainer.poseEstimator::resetFieldPosition));
        gamepad.bButton.onTrue(IntakeCommands.toggleHoodState());
    }

    @Override
    public void setupDisabledButtons() {
    }

    @Override
    public void setupTestButtons() {
    }

    public double getRightTriggerRaw() {
        return gamepad.getRawAxis(XboxController.Axis.kRightTrigger.value);
    }

    public double getLeftTriggerRaw() {
        return gamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    }
}
