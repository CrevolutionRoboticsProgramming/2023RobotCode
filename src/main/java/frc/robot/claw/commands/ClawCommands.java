package frc.robot.claw.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.claw.ClawConfig;

public class ClawCommands {
    public static Command setState(ClawConfig.ClawState state) {
        return new InstantCommand(() -> RobotContainer.claw.setClawState(state), RobotContainer.claw);
    }

    public static Command toggleState() {
        return new InstantCommand(() -> RobotContainer.claw.toggleClawState(), RobotContainer.claw); 
    }
}
