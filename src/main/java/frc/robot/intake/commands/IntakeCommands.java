package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.intake.IntakeConfig;

import java.util.function.DoubleSupplier;

public class IntakeCommands {
    @Deprecated
    public static Command setPivotState(IntakeConfig.PivotState state) {
//        return new InstantCommand(() -> RobotContainer.intakePivot.setPivotState(state), RobotContainer.intakePivot);
        return null;
    }

    @Deprecated
    public static Command setPivotOutput(DoubleSupplier supplier) {
        return new InstantCommand(() -> RobotContainer.intakePivot.setOutput(supplier.getAsDouble()), RobotContainer.intakePivot);
    }

    public static Command setHoodState(IntakeConfig.HoodState state) {
        return new InstantCommand(() -> RobotContainer.intakeHood.setState(state));
    }
}
