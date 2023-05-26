package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.intake.IntakeConfig.PivotState;
import frc.robot.intake.commands.IntakeCommands;
import frc.robot.intake.commands.SetPivotState;

public class ElevatorZero extends ParallelCommandGroup {
    public ElevatorZero() {
        // addCommands(
        //     new SetElevatorState(RobotContainer.elevator, ElevatorState.kZeroGoal)
        //     .alongWith(IntakeCommands.clearPivotDown())
        // );
    }
}
