package frc.robot;

import frc.robot.driver.DriverGamepad;
import frc.robot.intake.Intake;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.autos.OnePieceInside;
import frc.robot.commands.autos.TestAuton;
import frc.robot.commands.autos.TwoPieceInsideBalance;
import frc.robot.commands.autos.TwoPieceOutsideBalance;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.vision.PoseEstimator;

public class RobotContainer {
    /*Declare External Sensors -> Cameras */
    public static PhotonCamera orangePi = new PhotonCamera(VisionConstants.CAMERA_NAME);

    /*Declare Joystick*/
    public static XboxController driverControllerRetro;
    public static DriverGamepad driverGamepad;

    /*Declare Subsystems*/
    public static SwerveDrivetrain drivetrain;
    public static PoseEstimator poseEstimator;
    public static Intake intake;

    /*Sendable Chooser Selector for Auton */
    public static SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Subsystem initialization
        drivetrain = new SwerveDrivetrain();
        poseEstimator = new PoseEstimator(orangePi, drivetrain);
        intake = new Intake();

        // Gamepad initialization
        driverGamepad = new DriverGamepad();
        driverControllerRetro = new XboxController(JoystickConstants.DRIVER_PORT_ID);

        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driverControllerRetro,
                XboxController.Axis.kLeftY.value, XboxController.Axis.kLeftX.value, XboxController.Axis.kRightX.value
                , SwerveDrivetrainConstants.FIELD_RELATIVE, SwerveDrivetrainConstants.OPEN_LOOP));

        autoChooser = getAutonChooser();
        SmartDashboard.putData(autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /*Returns Auton Commands (Sendable Chooser) */
        return new TestAuton(drivetrain);
    }

    private SendableChooser<Command> getAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("TwoPieceINSIDEBalance", new TwoPieceInsideBalance(drivetrain));
        chooser.addOption("TwoPieceOUTSIDEBalance", new TwoPieceOutsideBalance(drivetrain));
        chooser.addOption("TestAuton", new TestAuton(drivetrain));
        chooser.addOption("OnePieceInside", new OnePieceInside(drivetrain));
        return chooser;
    }
}
