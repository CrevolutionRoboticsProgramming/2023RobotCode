package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.intake.IntakeConfig;

import java.util.function.DoubleSupplier;

public class IntakeCommands {
    private static IntakeConfig.HoodState hoodState = IntakeConfig.HoodState.kClosed;

    public static void setDefaultCommand() {
        var intake = RobotContainer.intake;
        intake.setDefaultCommand(new InstantCommand(() -> {
            // TODO: REMOVE THIS!
            intake.setRollerOutput(RobotContainer.driverGamepad.getRightTriggerRaw());
        }, intake));
    }

    public static Command setPivotState(IntakeConfig.PivotState state) {
        return new InstantCommand(() -> RobotContainer.intake.setPivotState(state), RobotContainer.intake);
    }

    @Deprecated
    public static Command setPivotOutput(DoubleSupplier supplier) {
        return new InstantCommand(() -> RobotContainer.intake.setPivotOutput(supplier.getAsDouble()), RobotContainer.intake);
    }

    public static Command setHoodState(IntakeConfig.HoodState state) {
        hoodState = state;
        return new InstantCommand(() -> RobotContainer.intake.setHoodState(state), RobotContainer.intake);
    }

    public static Command toggleHoodState() {
        if (hoodState == IntakeConfig.HoodState.kOpen) {
            hoodState = IntakeConfig.HoodState.kClosed;
        } else {
            hoodState = IntakeConfig.HoodState.kOpen;
        }

        return new InstantCommand(() -> RobotContainer.intake.setHoodState(hoodState), RobotContainer.intake);
    }
}
