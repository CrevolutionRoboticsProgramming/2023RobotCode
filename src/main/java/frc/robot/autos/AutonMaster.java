package frc.robot.autos;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
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
import frc.robot.drivetrain.commands.AntiSlip;

public final class AutonMaster {
    private static final PathConstraints GENERIC_PATH_CONSTRAINTS = new PathConstraints(3.5, 3);
    private static final PathConstraints SLOW_PATH_CONSTRAINTS = new PathConstraints(3, 2);
    private static final PathConstraints MEDIUM_PATH_CONSTRAINTS = new PathConstraints(3, 2);

    public static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
            Map.entry("Stop", new InstantCommand(RobotContainer.drivetrain::stopSwerve)),

            Map.entry("ZeroGyro", new InstantCommand(() -> {
                    RobotContainer.drivetrain.m_gyro.setYaw(RobotContainer.drivetrain.getYaw().getDegrees());
            })),

            Map.entry("IntakeDown", new ParallelCommandGroup(
                    new SetPivotState(RobotContainer.intakePivot, PivotState.kDeployed),
                    new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kCube)
            )),


            Map.entry("IntakeUp", new ParallelCommandGroup(
                    new SetPivotState(RobotContainer.intakePivot, PivotState.kStowed),
                    new InstantCommand(() -> RobotContainer.intakeRoller.stop(), RobotContainer.intakeRoller)
            )),

            Map.entry("ShootLow", new SequentialCommandGroup(
                    new SetPivotState(RobotContainer.intakePivot, PivotState.kScoreLow),
                    new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kOuttake).withTimeout(2)
            )),

            // Note: currently intake is positioned by drive team in auto due to slack in mechanism
            Map.entry("ShootMid", new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kOuttake).withTimeout(2)),

            Map.entry("ScoreMid", new SequentialCommandGroup(
                    ClawCommands.setState(ClawState.kClosed),
                    new ScoreMid(),
                    ClawCommands.setState(ClawState.kOpen)
            )),

            Map.entry("ScoreHigh", new SequentialCommandGroup(
                    ClawCommands.setState(ClawState.kClosed),
                    new ScoreHigh(),
                    ClawCommands.setState(ClawState.kOpen)
            )),

            Map.entry("Wait2Seconds", new WaitCommand(2)),

            Map.entry("HandoffCone", new HandoffCone()),

            Map.entry("HandoffCube", new HandoffCube()),

            Map.entry("ClearHood", new ClearHood()),

            Map.entry("ElevatorZero", new ElevatorZero()),

            Map.entry("AutoBalance", new AutoBalancing(RobotContainer.drivetrain)),

            Map.entry("AntiSlip", new AntiSlip(RobotContainer.drivetrain)),

            Map.entry("DropPiece", new InstantCommand(() -> {
                RobotContainer.claw.setClawState(ClawState.kOpen);
            }).andThen(new WaitCommand(2))),

            Map.entry("StopDrivetrain", new InstantCommand(() -> RobotContainer.drivetrain.stopSwerve()))
    ));

    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            RobotContainer.drivetrain::getPose,
            RobotContainer.drivetrain::resetOdometry,
            AutonConstants.AUTO_TRANSLATION_PID,
            AutonConstants.AUTO_ROTATION_PID,
            RobotContainer.drivetrain::setChassisSpeeds,
            eventMap,
            true,
            RobotContainer.drivetrain
    );

    private enum AutonConstraints {
        kGeneric(4, 3),
        kSlow(3, 2),
        kCreep(1, 1);

        AutonConstraints(double maxVelocity, double maxAcceleration) {
            constraints = new PathConstraints(maxVelocity, maxAcceleration);
        }

        PathConstraints get() {
            return constraints;
        }

        private final PathConstraints constraints;
    }

    /**
     * An enum containing all full auton paths along with a display name for each
     */
    public enum AutonPath {
        kClear1_5CubeMidBalance(
                "clear_1.5_mid_cube_balance",
                "Clear 1.5 Mid Cube + Balance",
                AutonConstraints.kGeneric.get(),
                AutonConstraints.kSlow.get()
        ),
        kClear1_5CubeMid(
                "clear_1.5_mid_cube_balance",
                "Clear 1.5 Mid Cube",
                AutonConstraints.kGeneric.get()
        ),
        kClear1_5HighConeBalance(
                "clear_1.5_high_cone_balance",
                "Clear 1.5 High Cone + Balance",
                AutonConstraints.kCreep.get(),
                AutonConstraints.kSlow.get(),
                AutonConstraints.kGeneric.get()
        );

        AutonPath(String pathName, String displayName, PathConstraints constraint, PathConstraints... constraints) {
            trajectory = PathPlanner.loadPathGroup(pathName, constraint, constraints);
            this.displayName = displayName;
        }

        public Command getFullAuton() {
            return autoBuilder.fullAuto(trajectory);
        }

        public String toString() {
            return displayName;
        }

        private final List<PathPlannerTrajectory> trajectory;
        private final String displayName;
    }

    public static SendableChooser<Command> getAutoSelector() {
        final var chooser = new SendableChooser<Command>();
        for (final var path : AutonPath.values()) {
            chooser.addOption(path.toString(), path.getFullAuton());
        }
        return chooser;
    }

    public AutonMaster() {
        throw new UnsupportedOperationException("Master Auton Class!");
    }
}
