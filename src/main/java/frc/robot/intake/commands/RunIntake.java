package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

import java.util.function.DoubleSupplier;

public class RunIntake extends CommandBase {
    private final DoubleSupplier supplier;
    private final Intake intake;

    public RunIntake(Intake intake, DoubleSupplier supplier) {
        this.supplier = supplier;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setRollerOutput(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerOutput(0);
    }
}
