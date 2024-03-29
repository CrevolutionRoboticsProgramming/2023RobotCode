// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.CrevoLib.math.Conversions;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveDrivetrainConstants {

        /*Swerve Drivetrain Construction */
        public static final boolean FIELD_RELATIVE = true;
        public static final boolean OPEN_LOOP = false;

        //Declaring CAN ID's for each module
        //Back Left -> Mod # 4
        //Back Right -> Mod # 1
        //Front Left -> Mod # 3
        //Front Right -> Mod # 2
        
        public static final int FRONT_RIGHT_DRIVE_ID = 10;
        public static final int FRONT_RIGHT_ANGLE_ID = 12;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;

        
        public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_ANGLE_ID = 3;
        public static final int FRONT_LEFT_CANCODER_ID = 2; 

        
        public static final int BACK_LEFT_DRIVE_ID = 4;
        public static final int BACK_LEFT_ANGLE_ID = 6;
        public static final int BACK_LEFT_CANCODER_ID = 5;

        
        public static final int BACK_RIGHT_DRIVE_ID = 7;
        public static final int BACK_RIGHT_ANGLE_ID = 9;
        public static final int BACK_RIGHT_CANCODER_ID = 8;

        //Pigeon Constants
        public static final int PIGEON_ID = 13;

        /* Module CANCoder Offsets */
        public static double FRONT_LEFT_OFFSET = 176.66; // 176.30
        public static double FRONT_RIGHT_OFFSET = 203.99; 
        public static double BACK_LEFT_OFFSET = 100.98;
        public static double BACK_RIGHT_OFFSET = 64.59;

        // public static double FRONT_LEFT_OFFSET = 176.66;
        // public static double FRONT_RIGHT_OFFSET = 207.42; 
        // public static double BACK_LEFT_OFFSET = 99.75;
        // public static double BACK_RIGHT_OFFSET = 67.93;

        /*Motor Invert Constants*/
        //Drive Motor Invert
        public static boolean FRONT_RIGHT_DRIVE_INVERT = true;
        public static boolean FRONT_LEFT_DRIVE_INVERT = true;
        public static boolean BACK_LEFT_DRIVE_INVERT = true;
        public static boolean BACK_RIGHT_DRIVE_INVERT = true;

        //Angle Motor Invert
        public static boolean FRONT_RIGHT_ANGLE_INVERT = false;
        public static boolean FRONT_LEFT_ANGLE_INVERT = false;
        public static boolean BACK_LEFT_ANGLE_INVERT = false;
        public static boolean BACK_RIGHT_ANGLE_INVERT = false;

        //CANCoder Invert
        public static boolean FRONT_RIGHT_CANCODER_INVERT = false;
        public static boolean FRONT_LEFT_CANCODER_INVERT = false;
        public static boolean BACK_LEFT_CANCODER_INVERT = false;
        public static boolean BACK_RIGHT_CANCODER_INVERT = false;

        //Pigeon Invert
        public static boolean PIGEON_INVERT = false;

        /*Drive Motor PID Values*/
        public static final double DRIVE_P = 0.08;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.1;
        public static final double DRIVE_F = 0.001;

        /*Angle Motor PID Values*/
        public static final double ANGLE_P = 0.09;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.1;
        public static final double ANGLE_F = 0.0;

        /*Drive Motor Characterization*/
        //Divide each by 12 to convert to volts for CTRE
        // public static final double DRIVE_kS = (0.19943 / 12);
        // public static final double DRIVE_kV = (0.10186 / 12);
        // public static final double DRIVE_kA = (0.016307 / 12);

        public static final double DRIVE_kS = (0.48665 / 12.0);
        public static final double DRIVE_kV = (2.4132 / 12.0);
        public static final double DRIVE_kA = (0.06921 / 12.0);

        /*Drive Motor Current Limiting*/
        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*Angle Motor Current Limiting*/
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25; 
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40; 
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        /*Neutral Modes*/
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;

        /*Swerve Gear Ratios*/
        //From SDS Website
        public static final double DRIVE_GEAR_RATIO = (6.75/1.0);
        public static final double ANGLE_GEAR_RATIO = ((-150.0/7)/1.0);
        
        /*Swerve Profiling Values*/
        public static final double MAX_SPEED = (Units.feetToMeters(18.0)); //Max from SDS Limit Speed
        public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12 * 0.5;

        /*Swerve Kinematics Constants*/
        public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(20.25);
        public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(28.75);
        public static final double DRIVETRAIN_ACTUAL_LENGTH = Units.inchesToMeters(28.0);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER  * Math.PI);

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
            new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
            new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
            new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));

        /*Open and Closed Loop Ramping*/
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double ANGLE_ALLOWABLE_CL_ERROR = Conversions.degreesToFalcon(3.0, Math.abs(ANGLE_GEAR_RATIO));
    }

    public static final class LoggingConstants {
        public static final boolean tuningMode = false;
    }

    public static enum DriveModes {
        CUSTOM_ROTATE, NORMAL
    }

    public static final class AutonConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);
    
        public static final double kPXController = 5;
        public static final double kDXController = 0.1;
        
        public static final double kPYController = 5;   
        public static final double kDYController = 0.1;
    
        public static final double kPThetaController = -2.78;
        public static final double kDThetaController = 0.001;

        public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(kPXController, 0, kDXController);
        public static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(kPThetaController, 0, kDThetaController);
    
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                8, 8);
      } 

      public static final class OnTheFlyLineupConstants {
        //taken through PathPlanner x/y positions
        public static final double[] LINEUP_Y_POSITIONS = new double[] 
        {4.97, 4.42, 3.85, 3.29, 2.75, 2.17, 1.62, 1.05, 0.51};
        public static final double FINAL_BLUE_X_POSITION = 1.80;
        public static final double FINAL_RED_X_POSITION = 14.75;

      }
    
    public static final Mode CUR_MODE = Mode.REAL;
    
    public static final class VisionConstants{
        public static final Transform3d CAM_TO_ROBOT = new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(-2.0), Units.inchesToMeters(11.5)),
                    new Rotation3d(0, 0, 0));
        public static final Transform3d ROBOT_TO_CAM = CAM_TO_ROBOT.inverse();
        
        public static final String CAMERA_NAME = "Arducam";

    }

    public static final class FieldConstants{ 
        public static final double FIELD_LENGTH = Units.feetToMeters(54.2708333);
        public static final double FIELD_WIDTH = Units.feetToMeters(26.291667);
    }

    public static final class PoseEstimationConstants {
        public static final Alliance CURRENT_ALLIANCE = Alliance.Red;
    }

    public static enum Mode {
        REAL, SIM, REPLAY;
    }

    public static final class JoystickConstants {
        public static final double STICK_DEADBAND = 0.1;
        public static final int DRIVER_PORT_ID = 0;
    }
    
    public static final class IntakeConstants {
        public static final int INTAKE_NEO_ID = 0;
    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID = 0;
        public static final int ENCODER_CHANNEL = 0;
    }
}
