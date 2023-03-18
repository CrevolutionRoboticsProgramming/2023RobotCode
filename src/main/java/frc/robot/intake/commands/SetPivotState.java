package frc.robot.intake.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakePivot;

public class SetPivotState extends CommandBase {
    private enum State {
        kProfile,
        kTension,
        kEnd
    }

    private final IntakePivot pivot;
    private final IntakeConfig.PivotState targetPivotState;

    private TrapezoidProfile profile;
    private final ArmFeedforward ffController;
    private final PIDController pidController;

    private Long startTs;
    private State state;

    public SetPivotState(IntakePivot pivot, IntakeConfig.PivotState targetPivotState) {
        this.pivot = pivot;
        this.targetPivotState = targetPivotState;

        ffController = new ArmFeedforward(IntakeConfig.kS, IntakeConfig.kG, IntakeConfig.kV, IntakeConfig.kA);
        pidController = new PIDController(IntakeConfig.kVelP, IntakeConfig.kVelI, IntakeConfig.kVelD);

        startTs = null;
        state = State.kProfile;

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

        state = State.kProfile;
    }

    @Override
    public void execute() {
        switch (state) {
            case kProfile:
                final var duration = getElapsedTime();
                runProfile(duration);
                if (profile.isFinished(duration)) {
                    state = (targetPivotState == IntakeConfig.PivotState.kDeployed) ? State.kTension : State.kEnd;

                    // TODO: Remove debug print
                    if (state == State.kTension) {
                        System.out.println(getName() + ": starting tensioning");
                    }
                }
                break;

            case kTension:
                pivot.set(IntakeConfig.kTensionOutput);
                if (pivot.getOutputCurrent() >= IntakeConfig.kTensionCurrentLimit) {
                    state = State.kEnd;
                }
                break;

            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return state == State.kEnd;
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    private void runProfile(double duration) {
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var targetState = profile.calculate(duration);

        final var ffOutput = ffController.calculate(pivot.getAngleRads(), targetState.velocity);
        final var pidOutput = pidController.calculate(pivot.getVelocityRps(), targetState.velocity);

        pivot.set((ffOutput + pidOutput) / 12.0);
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
