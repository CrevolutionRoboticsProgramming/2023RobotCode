package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;

class HoldElevatorState extends CommandBase {
    private final Elevator elevator;
    private ElevatorConfig.ElevatorState targetState;

    HoldElevatorState(Elevator elevator) {
        this.elevator = elevator;
        targetState = ElevatorConfig.ElevatorState.kUnspecified;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        targetState = elevator.getState();
        System.out.println("[" + getName() + "]: " + targetState.name());
    }

    @Override
    public void execute() {
        final var limitStates = elevator.getLimitStates();
        switch (targetState) {
            case kZeroGoal:
                elevator.setElevatorOutput(-ElevatorConfig.kSeekVoltage);
                if (limitStates[0]) {
                    elevator.setState(ElevatorConfig.ElevatorState.kZero);
                    targetState = ElevatorConfig.ElevatorState.kZero;
                }
                break;
            case kHighGoal:
                elevator.setElevatorOutput(ElevatorConfig.kSeekVoltage);
                if (limitStates[1]) {
                    elevator.setState(ElevatorConfig.ElevatorState.kHigh);
                    targetState = ElevatorConfig.ElevatorState.kHigh;
                }
                break;
            default:
                elevator.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
