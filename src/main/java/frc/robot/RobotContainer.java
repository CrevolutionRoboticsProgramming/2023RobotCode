package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.intake.IntakeHood;

import frc.robot.driver.DriverGamepad;
import frc.robot.intake.IntakePivot;
import frc.robot.intake.IntakeRoller;
import frc.robot.intake.commands.HoldPivot;
import frc.robot.operator.OperatorGamepad;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.autos.AutonMaster;
import frc.robot.claw.Claw;
import frc.robot.drivetrainOld.SwerveDrivetrain;
import frc.robot.drivetrainOld.commands.TeleopDrive;
import frc.robot.elevator.Elevator;


public class RobotContainer {

//  public static PhotonCamera orangePi = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

    /*Declare Joystick*/
    public static XboxController driverControllerRetro = new XboxController(0);
    public static DriverGamepad driverGamepad;
    public static OperatorGamepad operatorGamepad;

    /*Declare Subsystems*/
    public static Drivetrain drivetrain;
//    public static PoseEstimator poseEstimator;
    public static IntakePivot intakePivot;
    public static IntakeRoller intakeRoller;
    public static IntakeHood intakeHood;
    public static Elevator elevator;
    public static Claw claw;
    //public static PneumaticsControlModule pcm;

    /*Sendable Chooser Selector for Auton */
    public static SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Subsystem initialization
        drivetrain = new Drivetrain();
//        poseEstimator = new PoseEstimator(orangePi, drivetrain);
        intakePivot = new IntakePivot();
        intakeRoller = new IntakeRoller();
        intakeHood = new IntakeHood();
        elevator = new Elevator();
        claw = new Claw();

    

        // Gamepad initialization
        driverGamepad = new DriverGamepad();
        operatorGamepad = new OperatorGamepad();


       autoChooser = getAutonChooser();
       ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Chooser");
       autonTab.add(autoChooser);
       SmartDashboard.putData(autoChooser);

        setDefaultCommands();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /*Returns Auton Commands (Sendable Chooser) */
//        return autoChooser.getSelected();
        return AutonMaster.midBalance();
    }

    private SendableChooser<Command> getAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Blue2PieceBalance", AutonMaster.bluePreloadPickupScoreBalance());
        chooser.addOption("BlueNoElevator2PieceBalance", AutonMaster.blueNoElevatorPreloadPickupScoreBalance());
        chooser.addOption("Blue1PieceBalance", AutonMaster.bluePreloadCommunityBalance());
        chooser.addOption("Blue1PiecePickupBalance", AutonMaster.blueMiddlePreloadPickupBalance());
        chooser.addOption("Red2PieceBalance", AutonMaster.redPreloadPickupScoreBalance());
        chooser.addOption("RedNoElevator2PieceBalance", AutonMaster.redNoElevatorPreloadPickupScoreBalance());
        chooser.addOption("RedPreloadPieceCommunityBalance", AutonMaster.redPreloadCommunityBalance()); 
        chooser.addOption("Red1PiecePickupBalance", AutonMaster.redMiddlePreloadPickupBalance());
        chooser.addOption("TestAutoBalance", AutonMaster.testAutoBalance());
        chooser.addOption("BlueScoreLowMobility", AutonMaster.blueScoreLowMobility());
        return chooser;
    }

    public void reset() {
        CommandScheduler.getInstance().clearButtons();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        driverGamepad.resetConfig();
        operatorGamepad.resetConfig();
    }

    private void setDefaultCommands() {
        intakePivot.setDefaultCommand(new HoldPivot(intakePivot));
        elevator.setDefaultCommand(ElevatorCommands.holdState());
        drivetrain.setDefaultCommand(DrivetrainCommands.driveFieldOriented(
                driverGamepad::getDriveTranslationX,
                driverGamepad::getDriveTranslationY,
                driverGamepad::getDriveRotation
        ));
    }
}
