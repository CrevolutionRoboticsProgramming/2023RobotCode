package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.commands.SetElevatorState;

public class ScoreHigh extends SequentialCommandGroup {
    public ScoreHigh() {
        addCommands(
                new SetElevatorState(RobotContainer.elevator, ElevatorConfig.ElevatorState.kHighGoal)
                        .alongWith(new WaitCommand(0.5).andThen(new ClearHood()))
        );
    }
}
