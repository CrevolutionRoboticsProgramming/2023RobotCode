package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.claw.ClawConfig;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.SetPivotState;

public class HandoffCube extends SequentialCommandGroup {
    public HandoffCube() {
        addCommands(
                new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kNeutral)
                        .alongWith(ClawCommands.setState(ClawConfig.ClawState.kOpen)),
                new SetElevatorState(RobotContainer.elevator, ElevatorConfig.ElevatorState.kLoad),
                new InstantCommand(() -> RobotContainer.intakeHood.setState(IntakeConfig.HoodState.kOpen)),
                new WaitCommand(0.25),
                new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kHandoff).withTimeout(0.5),
//                new WaitCommand(1),
                new SetElevatorState(RobotContainer.elevator, ElevatorConfig.ElevatorState.kZero),
                ClawCommands.setState(ClawConfig.ClawState.kClosed),
                new SetElevatorState(RobotContainer.elevator, ElevatorConfig.ElevatorState.kPreScore)
        );
    }
}
