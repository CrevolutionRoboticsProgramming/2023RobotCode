package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.commands.SetPivotState;

public class ClearHood extends ParallelCommandGroup {
    public ClearHood() {
        addCommands(
                new SetPivotState(RobotContainer.intakePivot, IntakeConfig.PivotState.kNeutral),
                new InstantCommand(() -> RobotContainer.intakeHood.setState(IntakeConfig.HoodState.kClosed))
        );
    }
}
