package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakeHood;

public class ToggleHood extends CommandBase {
    private final IntakeHood hood;

    public ToggleHood(IntakeHood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public String getName() {
        return "ToggleHood";
    }

    @Override
    public void initialize() {
        final IntakeConfig.HoodState newState;
        if (hood.getState() == IntakeConfig.HoodState.kOpen) {
            newState = IntakeConfig.HoodState.kClosed;
        } else {
            newState = IntakeConfig.HoodState.kOpen;
        }

        hood.setState(newState);

        System.out.println(getName() + ": set state " + newState.name());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
