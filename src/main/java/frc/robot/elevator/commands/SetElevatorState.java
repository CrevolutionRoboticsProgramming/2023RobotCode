package frc.robot.elevator.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.ElevatorConfig.ElevatorState;

public class SetElevatorState extends CommandBase{
    private final Elevator elevator;

    private final ElevatorState targetState;

    private TrapezoidProfile profile;
    private final ElevatorFeedforward ffController;
    private final PIDController pidController;

    private Long startTs; 

    public SetElevatorState(Elevator elevator, ElevatorState targetElevatorState) {
        this.elevator = elevator;
        this.targetState = targetElevatorState;

        ffController = new ElevatorFeedforward(ElevatorConfig.kS,ElevatorConfig.kG, ElevatorConfig.kV, ElevatorConfig.kA);
        pidController = new PIDController(ElevatorConfig.kP, ElevatorConfig.kI, ElevatorConfig.kD);
        
        startTs = null;

        addRequirements(this.elevator);
    }

    @Override
    public String getName() {
        return "SetElevatorState[" + targetState.name() + "]";
    }

    @Override
    public void initialize() {
        System.out.println(getName() + ": init");

        profile = generateElevatorProfile();
        
        startTs = null;
    }

    @Override
    public void execute() {
        final var duration = getElapsedTime();
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var targetState = profile.calculate(duration);

        final var ffOutput = ffController.calculate(targetState.velocity);
        final var pidOutput = pidController.calculate(elevator.getVelocityMps(), targetState.velocity);

        final var voltage = ffOutput + pidOutput;

//        System.out.println("[" + getName() + "] velocity (" + elevator.getVelocityMps() + "), target (" + targetState.velocity + "), voltage (" + voltage + ")");
        elevator.setElevatorOutput(ffOutput + pidOutput);
    }


    @Override
    public void end(boolean interrupted) {
        System.out.println(getName() + ": end");
        boolean[] elevatorLimitSwitchStates = elevator.getLimitStates();
    
        if (elevatorLimitSwitchStates[0]) {
            elevator.setState(ElevatorState.kZero);
        } else if (elevatorLimitSwitchStates[1]) {
            elevator.setState(ElevatorState.kHigh);
        } else {
            elevator.setState(interrupted ? ElevatorState.kUnspecified : targetState);
        }

        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        final var limitStates = elevator.getLimitStates();
        return (limitStates[0] && elevator.getVelocityMps() < 0)
                || (limitStates[1] && elevator.getVelocityMps() > 0)
                || (profile != null && profile.isFinished(getElapsedTime()));
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }


    private TrapezoidProfile generateElevatorProfile() {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConfig.kMaxVelocity,
                ElevatorConfig.kMaxAcceleration
            ),
            new TrapezoidProfile.State(targetState.target, 0),  //goal state
            new TrapezoidProfile.State(elevator.getPositionMeters(), elevator.getVelocityMps())); //start state
    }
}
