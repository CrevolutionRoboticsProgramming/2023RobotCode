package frc.robot.autos;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.intake.IntakePivot;
import frc.robot.intake.IntakeConfig.PivotState;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.SetPivotState;
import frc.robot.claw.ClawConfig.ClawState;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.commands.ClearHood;
import frc.robot.commands.ElevatorZero;
import frc.robot.commands.HandoffCone;
import frc.robot.commands.HandoffCube;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreMid;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.SetElevatorState;

public final class AutonMaster {

    private static final PathConstraints GENERIC_PATH_CONSTRAINTS = new PathConstraints(4, 3);

    private static final PathConstraints SLOW_PATH_CONSTRAINTS = new PathConstraints(2, 1);

    private static final PathConstraints MEDIUM_PATH_CONSTRAINTS = new PathConstraints(3, 2);
    public static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("Stop", new InstantCommand(RobotContainer.drivetrain::stopSwerve)),
        
        Map.entry("ZeroGyro", new InstantCommand(() -> RobotContainer.drivetrain.m_gyro.setYaw(RobotContainer.drivetrain.getYaw().getDegrees()))),
        
        Map.entry("IntakeDown", 
        new ParallelCommandGroup(
            new SetPivotState(RobotContainer.intakePivot, PivotState.kDeployed), 
            new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kCube))),

        
        Map.entry("IntakeUp", 
        new SetPivotState(RobotContainer.intakePivot, PivotState.kStowed)),
        

        Map.entry("ScoreHigh", new SequentialCommandGroup(
            ClawCommands.setState(ClawState.kClosed),   
            new ScoreHigh(),
            ClawCommands.setState(ClawState.kOpen)
            )),

        Map.entry("ScoreMid", new SequentialCommandGroup(
            ClawCommands.setState(ClawState.kClosed),   
            new ScoreHigh(),
            ClawCommands.setState(ClawState.kOpen)
            )),

        Map.entry("ScoreLow", new SequentialCommandGroup(
//            new SetPivotState(RobotContainer.intakePivot, PivotState.kScoreLow),
            new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kOuttake).withTimeout(2)
        )),
    
        Map.entry("Wait2Seconds", new WaitCommand(2)),


        Map.entry("HandoffCone", new HandoffCone()),

        Map.entry("HandoffCube", new HandoffCube()),

        Map.entry("ClearHood", new ClearHood()),
        
        Map.entry("ElevatorZero", new ElevatorZero()),

        Map.entry("AutoBalance", new AutoBalancing(RobotContainer.drivetrain)),
        
        Map.entry("AntiSlip", new AntiSlip(RobotContainer.drivetrain)),

        Map.entry("ElevatorHigh", new SetElevatorState(RobotContainer.elevator, ElevatorState.kHigh)),
        Map.entry("DropPiece", new InstantCommand(() -> RobotContainer.claw.setClawState(ClawState.kOpen)).andThen(new WaitCommand(2)))

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
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestBlueEventMap", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command bluePreloadCommunityBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BluePreloadCommunityBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command bluePreloadPickupScoreBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BluePreloadPickupScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command blueMiddlePreloadPickupBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueMiddlePreloadPickupBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command blueNoElevatorPreloadPickupScoreBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueNoElevatorPreloadPickupScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command redPreloadCommunityBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedPreloadCommunityBalance", new PathConstraints(2, 1.5)));
    }

    public static Command redPreloadPickupScoreBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedPreloadPickupScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command redMiddlePreloadPickupBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedMiddlePreloadPickupBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command redNoElevatorPreloadPickupScoreBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedNoElevatorPreloadPickupScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command testAutoBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("TestAutoBalance", SLOW_PATH_CONSTRAINTS));
    }

    public static Command blueScoreLowMobility() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueScoreLowMobility", new PathConstraints(3, 2)));
    }

    public static Command redScoreLowMobility() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedScoreLowMobility", new PathConstraints(3, 2)));
    }

    public static Command blueNoElevatorOneandHalfPiece() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueNoElevator1.5Piece", MEDIUM_PATH_CONSTRAINTS));
    }

    public static Command blueOneandHalfPieceBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("blueOneandHalfPieceBalance", SLOW_PATH_CONSTRAINTS));
    }

    public static Command blueRightScoreLowMobility() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueRightScoreLowMobility", new PathConstraints(3, 1.5)));
    }

    public static Command blueLeft1_5CubeMid() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueLeft1.5CubeMid", new PathConstraints(3, 2)));
    }

    public static Command blusCenterLowScoreBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueCenterLowScoreBalance", new PathConstraints(2, 2)));
    }

    public static Command redBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedBalance", new PathConstraints(2, 2)));
    }

    public static Command midBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueCenterMidScoreBalance", new PathConstraints(1, 1)));
    }

    public AutonMaster() {
        throw new UnsupportedOperationException("Master Auton Class!");
    }


}
