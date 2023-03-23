package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.ElevatorConfig.ElevatorState;

public class ElevatorCommands {
    public static Command setState(ElevatorConfig.ElevatorState state) {
        return new SetElevatorState(RobotContainer.elevator, state);
    }

    public static Command holdState() {
        return new HoldElevatorState(RobotContainer.elevator);
    }
}
