package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakeRoller;

import java.util.function.Supplier;

public class RunIntake extends CommandBase {
    public enum Mode {
        kCube(IntakeConfig.kCubeProfile), kCone(IntakeConfig.kConeProfile), kOuttake(IntakeConfig.kOuttake);

        Mode(IntakeConfig.IntakeProfile profile) {
            kProfile = profile;
        }

        private final IntakeConfig.IntakeProfile kProfile;
    }

    private final IntakeRoller roller;
    private final Supplier<Mode> mode;

    private Mode lastMode;

    public RunIntake(IntakeRoller roller, Supplier<Mode> modeSupplier) {
        this.roller = roller;
        this.mode = modeSupplier;

        addRequirements(roller);
    }

    @Override
    public String getName() {
        return "RunIntake";
    }

    @Override
    public void initialize() {
        lastMode = mode.get();
        roller.setRollerProfile(lastMode.kProfile);
        roller.setRollerOutput(1);

        System.out.println(getName() + ": setting mode to " + lastMode.name());
    }

    @Override
    public void execute() {
        final var currentMode = mode.get();
        if (lastMode != currentMode) {
            roller.setRollerProfile(currentMode.kProfile);
            lastMode = currentMode;

            System.out.println(getName() + ": setting mode to " + lastMode.name());
        }
    }

    @Override
    public void end(boolean interrupted) {
        roller.stop();
    }
}
