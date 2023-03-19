package frc.robot;

import frc.robot.intake.IntakeHood;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.driver.DriverGamepad;
import frc.robot.intake.IntakePivot;
import frc.robot.intake.IntakeRoller;
import frc.robot.intake.commands.HoldPivot;
import frc.robot.operator.OperatorGamepad;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.autos.AutonMaster;
import frc.robot.autos.OnePieceInside;
import frc.robot.autos.TestAuton;
import frc.robot.autos.TwoPieceInsideBalance;
import frc.robot.autos.TwoPieceOutsideBalance;
import frc.robot.claw.Claw;
import frc.robot.drivetrain.SwerveDrivetrain;
import frc.robot.drivetrain.commands.TeleopDrive;
import frc.robot.elevator.Elevator;
import frc.robot.vision.PoseEstimator;


public class RobotContainer {

//  public static PhotonCamera orangePi = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

    /*Declare Joystick*/
    public static XboxController driverControllerRetro = new XboxController(0);
    public static DriverGamepad driverGamepad;
    public static OperatorGamepad operatorGamepad;

    /*Declare Subsystems*/
    public static SwerveDrivetrain drivetrain;
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
        drivetrain = new SwerveDrivetrain();
//        poseEstimator = new PoseEstimator(orangePi, drivetrain);
        intakePivot = new IntakePivot();
        intakeRoller = new IntakeRoller();
        intakeHood = new IntakeHood();
        elevator = new Elevator();
        claw = new Claw();

        // Gamepad initialization
        driverGamepad = new DriverGamepad();
        operatorGamepad = new OperatorGamepad();

        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driverControllerRetro,
                XboxController.Axis.kLeftY.value, XboxController.Axis.kLeftX.value, XboxController.Axis.kRightX.value
                , SwerveDrivetrainConstants.FIELD_RELATIVE, SwerveDrivetrainConstants.OPEN_LOOP));

//        autoChooser = getAutonChooser();
//        SmartDashboard.putData(autoChooser);

        setDefaultCommands();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /*Returns Auton Commands (Sendable Chooser) */
        return getAutonChooser().getSelected();
    }

    private SendableChooser<Command> getAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("TwoPieceINSIDEBalance", new TwoPieceInsideBalance(drivetrain));
        chooser.addOption("TwoPieceOUTSIDEBalance", new TwoPieceOutsideBalance(drivetrain));
        chooser.addOption("TestAuton", new TestAuton(drivetrain));
        chooser.addOption("OnePieceInside", new OnePieceInside(drivetrain));
        chooser.addOption("TestMarkerEvents", AutonMaster.testAutoBlue());
        return chooser;
    }

    public void reset() {
        driverGamepad.resetConfig();
        operatorGamepad.resetConfig();
    }

    private void setDefaultCommands() {
        intakePivot.setDefaultCommand(new HoldPivot(intakePivot));
    }
}
