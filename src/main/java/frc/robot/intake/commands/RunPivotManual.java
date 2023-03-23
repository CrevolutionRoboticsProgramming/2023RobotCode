package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakePivot;

import java.util.function.DoubleSupplier;

public class RunPivotManual extends CommandBase {
    private final IntakePivot pivot;
    private final DoubleSupplier supplier;

    public RunPivotManual(IntakePivot pivot, DoubleSupplier supplier) {
        this.pivot = pivot;
        this.supplier = supplier;

        addRequirements(pivot);
    }

    @Override
    public String getName() {
        return getClass().getName();
    }

    @Override
    public void initialize() {
        pivot.setState(IntakeConfig.PivotState.kUnspecified);
    }

    @Override
    public void execute() {
        final var input = supplier.getAsDouble();
        if (input < 0 && pivot.getLimitSwitchState()) {
            pivot.stop();
        } else {
            pivot.setOutput(input);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setState(pivot.getLimitSwitchState() ? IntakeConfig.PivotState.kDeployed : IntakeConfig.PivotState.kUnspecified);
        pivot.stop();
    }
}
