package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakeRoller;

public class RunIntake extends CommandBase {
    public enum Mode {
        kCube(IntakeConfig.kCubeProfile), kCone(IntakeConfig.kConeProfile), kOuttake(IntakeConfig.kOuttake);

        Mode(IntakeConfig.IntakeProfile profile) {
            kProfile = profile;
        }

        private final IntakeConfig.IntakeProfile kProfile;
    }

    private final IntakeRoller roller;
    private final Mode mode;

    public RunIntake(IntakeRoller roller, Mode mode) {
        this.roller = roller;
        this.mode = mode;

        addRequirements(roller);
    }

    @Override
    public String getName() {
        return "RunIntake[" + mode.name() + "]";
    }

    @Override
    public void initialize() {
        roller.setRollerProfile(mode.kProfile);
        roller.setRollerOutput(1);
    }

    @Override
    public void end(boolean interrupted) {
        roller.stop();
    }
}
