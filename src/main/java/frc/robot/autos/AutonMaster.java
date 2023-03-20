package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.intake.IntakeConfig.PivotState;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.SetPivotState;

public final class AutonMaster {
    
    public static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("Stop", new InstantCommand(RobotContainer.drivetrain::stopSwerve)),
        Map.entry("ZeroGyro", new InstantCommand(() -> RobotContainer.drivetrain.pigeon.setYaw(RobotContainer.drivetrain.getYaw().getDegrees()))),
        Map.entry("IntakeDown", new ParallelCommandGroup(new SetPivotState(RobotContainer.intakePivot, PivotState.kDeployed), new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kCube))),
        Map.entry("IntakeUp", new SetPivotState(RobotContainer.intakePivot, PivotState.kHumanPlayer)),
        Map.entry("AutoBalance", new AutoBalancing(RobotContainer.drivetrain)),
        Map.entry("AntiSlip", new AntiSlip(RobotContainer.drivetrain))
    ));

    private static final SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
        RobotContainer.drivetrain::getPose, 
        RobotContainer.drivetrain::resetOdometry,
        AutonConstants.AUTO_TRANSLATION_PID,
        AutonConstants.AUTO_ROTATION_PID, 
        RobotContainer.drivetrain::setChassisSpeeds, 
        eventMap, 
        false, 
        RobotContainer.drivetrain);



    public static Command testAutoBlue() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestBlueEventMap", new PathConstraints(4, 3)));
    }

    public AutonMaster() {
        throw new UnsupportedOperationException("Master Auton Class!");
    }


}
