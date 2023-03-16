package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakeRoller;

import java.util.function.DoubleSupplier;

public class RunIntakeManual extends CommandBase {
    private final DoubleSupplier supplier;
    private final IntakeRoller roller;

    public RunIntakeManual(IntakeRoller roller, DoubleSupplier supplier) {
        this.supplier = supplier;
        this.roller = roller;

        addRequirements(roller);
    }

    @Override
    public String getName() {
        return "Intake.RunIntakeManual";
    }

    @Override
    public void initialize() {
        roller.setRollerProfile(IntakeConfig.kDefaultProfile);
    }

    @Override
    public void execute() {
        roller.setRollerOutput(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        roller.setRollerOutput(0);
    }
}
