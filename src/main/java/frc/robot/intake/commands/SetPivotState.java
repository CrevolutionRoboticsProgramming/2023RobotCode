package frc.robot.intake.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakePivot;

public class SetPivotState extends CommandBase {
    private final IntakePivot pivot;
    private final IntakeConfig.PivotState targetPivotState;

    private TrapezoidProfile profile;
    private final ArmFeedforward ffController;
    private final PIDController pidController;

    private Long startTs;

    public SetPivotState(IntakePivot pivot, IntakeConfig.PivotState targetPivotState) {
        this.pivot = pivot;
        this.targetPivotState = targetPivotState;

        ffController = new ArmFeedforward(IntakeConfig.kS, IntakeConfig.kG, IntakeConfig.kV, IntakeConfig.kA);
        pidController = new PIDController(IntakeConfig.kVelP, IntakeConfig.kVelI, IntakeConfig.kVelD);

        startTs = null;

        addRequirements(pivot);
    }

    @Override
    public String getName() {
        return "SetPivotState[" + targetPivotState.name() + "]";
    }

    @Override
    public void initialize() {
        System.out.println(getName() + ": init");
        profile = generateProfile();

        startTs = null;
    }

    @Override
    public void execute() {
        final var duration = getElapsedTime();
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var targetState = profile.calculate(duration);

        final var ffOutput = ffController.calculate(pivot.getAngleRads(), targetState.velocity);
        final var pidOutput = pidController.calculate(pivot.getVelocityRps(), targetState.velocity);

        pivot.setOutput((ffOutput + pidOutput) / 12.0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(getName() + ": end");

        final IntakeConfig.PivotState state;
        if (pivot.getLimitSwitchState()) {
            state = IntakeConfig.PivotState.kDeployed;
        } else if (interrupted) {
            state = IntakeConfig.PivotState.kUnspecified;
        } else {
            state = targetPivotState;
        }

        pivot.setState(state);
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        final var time = getElapsedTime();

        // 1) Profile is still running and reached limit switch
        return (profile != null && profile.calculate(time).velocity < 0 && pivot.getLimitSwitchState()) ||
                (profile != null && profile.isFinished(time));
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    private TrapezoidProfile generateProfile() {
        final var startPos = pivot.getAngleRads();
        final var endPos = targetPivotState.target;

        final var distance = Math.min(Math.abs(endPos - startPos), (2 * Math.PI) - Math.abs(endPos - startPos));

        System.out.println("Start: " + startPos + ", End: " + endPos + ", Dist: " + distance);

        final TrapezoidProfile.State startState, goalState;
        if (startPos < endPos) {
            goalState = new TrapezoidProfile.State(distance, 0.0);
            startState = new TrapezoidProfile.State(0.0, pivot.getVelocityRps());
        } else {
            goalState = new TrapezoidProfile.State(0.0, 0.0);
            startState = new TrapezoidProfile.State(distance, pivot.getVelocityRps());
        }

        return new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        IntakeConfig.kMaxAngularVelocity,
                        IntakeConfig.kMaxAngularAcceleration
                ),
                goalState,
                startState
        );
    }
}
