package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakePivot;

public class HoldPivot extends CommandBase {
    private final IntakePivot pivot;

    public HoldPivot(IntakePivot pivot) {
        this.pivot = pivot;

        addRequirements(pivot);
    }

    // TODO: Determine if positional PID is required to hold position or if break mode is sufficient
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}
