package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    public void execute() {
        System.out.println(supplier.getAsDouble());
        pivot.set(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}
