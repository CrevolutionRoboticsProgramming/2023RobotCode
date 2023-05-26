package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.intake.IntakeConfig.HoodState;
import frc.robot.intake.IntakeConfig.PivotState;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.SetPivotState;
import frc.robot.intake.commands.RunIntake.Mode;

public class ShootHighTeleop extends SequentialCommandGroup{
    public ShootHighTeleop() {
        // addCommands(
        //     new ParallelCommandGroup(new SetElevatorState(RobotContainer.elevator, ElevatorState.kChamber), 
        //     new SetPivotState(RobotContainer.intakePivot, PivotState.kShootHigh)),
        //     new InstantCommand(() -> RobotContainer.intakeHood.setState(HoodState.kOpen)).alongWith(new WaitCommand(1)),
        //     new ParallelCommandGroup(new RunIntake(RobotContainer.intakeRoller, Mode.kShooter).withTimeout(1.5),
        //     new InstantCommand( () -> RobotContainer.intakeHood.setState(HoodState.kClosed)))
        // );
    }
}
