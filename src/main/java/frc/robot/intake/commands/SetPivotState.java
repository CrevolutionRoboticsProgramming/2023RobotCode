package frc.robot.intake.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakePivot;

public class SetPivotState extends CommandBase {
    private final IntakePivot pivot;
    private final IntakeConfig.PivotState state;

    private TrapezoidProfile profile;
    private final ArmFeedforward ffController;
    private final PIDController pidController;

    private Long startTs;

    public SetPivotState(IntakePivot pivot, IntakeConfig.PivotState state) {
        this.pivot = pivot;
        this.state = state;

        ffController = new ArmFeedforward(IntakeConfig.kS, IntakeConfig.kG, IntakeConfig.kV, IntakeConfig.kA);
        pidController = new PIDController(IntakeConfig.kVelP, IntakeConfig.kVelI, IntakeConfig.kVelD);

        startTs = null;

        addRequirements(pivot);
    }

    @Override
    public String getName() {
        return "SetPivotState[" + state.name() + "]";
    }

    @Override
    public void initialize() {
        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        IntakeConfig.kMaxAngularVelocity,
                        IntakeConfig.kMaxAngularAcceleration
                ),
                new TrapezoidProfile.State(pivot.getAngleRads(), pivot.getVelocityRps()),
                new TrapezoidProfile.State(state.target, 0.0)
        );
    }

    @Override
    public void execute() {
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var duration = getElapsedTime();
        final var targetState = profile.calculate(duration);

        final var ffOutput = ffController.calculate(pivot.getAngleRads(), pivot.getVelocityRps());
        final var pidOutput = pidController.calculate(pivot.getVelocityRps(), targetState.velocity);

        pivot.setPivotOutput(ffOutput + pidOutput);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return profile != null && profile.isFinished(getElapsedTime());
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis()) / 1000;
    }
}
