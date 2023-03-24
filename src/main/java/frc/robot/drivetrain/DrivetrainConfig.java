package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.CrevoLib.math.Conversions;

public class DrivetrainConfig {
    // Physical Constants
    public static final double kDriveGearRatio = (6.75/1.0);
    public static final double kAngleGearRatio = ((-150.0/7)/1.0);

    public static final double kDrivetrainWidth = Units.inchesToMeters(20.25);
    public static final double kDrivetrainHeight = Units.inchesToMeters(28.75);
    public static final double kDrivetrainActualLength = Units.inchesToMeters(28.0);
    public static final double kWheelDiameter = Units.inchesToMeters(4.0);
    public static final double kWheelCircumfurence = (kWheelDiameter * Math.PI);

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            new Translation2d(kDrivetrainHeight / 2.0, kDrivetrainWidth / 2.0),
            new Translation2d(kDrivetrainHeight / 2.0, -kDrivetrainWidth / 2.0),
            new Translation2d(-kDrivetrainHeight / 2.0, kDrivetrainWidth / 2.0),
            new Translation2d(-kDrivetrainHeight / 2.0, -kDrivetrainWidth / 2.0));

    // Devices
    public static final SwerveModule.Config kFrontLeftModuleConfig = new SwerveModule.Config(
            1,
            3,
            2,
            Rotation2d.fromDegrees(176.66)
    );

    public static final SwerveModule.Config kFrontRightModuleConfig = new SwerveModule.Config(
            10,
            12,
            11,
            Rotation2d.fromDegrees(203.99)
    );

    public static final SwerveModule.Config kBackLeftModuleConfig = new SwerveModule.Config(
            4,
            6,
            5,
            Rotation2d.fromDegrees(100.98)
    );

    public static final SwerveModule.Config kBackRightModuleConfig = new SwerveModule.Config(
            7,
            9,
            8,
            Rotation2d.fromDegrees(64.59)
    );

    public static final int kPigeonId = 13;
    public static final boolean kPigeonInvert = false;

    // Controller Constants
    public static final double kDriveS = (0.48665 / 12.0);
    public static final double kDriveV = (2.4132 / 12.0);
    public static final double kDriveA = (0.06921 / 12.0);

    public static final double kDriveP = 0.01; // 0.08;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0; // 0.1;

    public static final double kAngleP = 0.09;
    public static final double kAngleI = 0.0;
    public static final double kAngleD = 0.1;

    public static final double kMaxVelocity = Units.feetToMeters(16);
    public static final double kMaxAngularVelocity = Math.PI * 4.12 * 0.4;

    public static final double kAngleAllowableClosedLoopError = Conversions.degreesToFalcon(3.0, Math.abs(kAngleGearRatio));

    /*Neutral Modes*/
    public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;
    public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;

    /*Open and Closed Loop Ramping*/
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    public static CANCoderConfiguration getCANCoderConfiguration(boolean inverted) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = inverted;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        return config;
    }

    public static TalonFXConfiguration getDriveMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
        config.slot0.kP = kDriveP;
        config.slot0.kI = kDriveI;
        config.slot0.kD = kDriveD;
        config.slot0.kF = 0;
        config.supplyCurrLimit = limit;
        config.openloopRamp = kOpenLoopRamp;
        config.closedloopRamp = kClosedLoopRamp;
        return config;
    }

    public static TalonFXConfiguration getAngleMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
        config.slot0.kP = kAngleP;
        config.slot0.kI = kAngleI;
        config.slot0.kD = kAngleD;
        config.slot0.kF = 0;
        config.supplyCurrLimit = limit;
        return config;
    }

    public static final double kSlowModeTranslationModifier = 0.5;
    public static final double kSlowModeRotationModifier = 0.5;
}
