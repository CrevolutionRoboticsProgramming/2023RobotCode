package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.CrevoLib.defaultconfigs.CTREConfigs;
import frc.robot.CrevoLib.math.Conversions;
import frc.robot.CrevoLib.util.CTREModuleState;


public class SwerveModule {
    private final int moduleId;

    private final TalonFX angleFalcon;
    private final TalonFX driveFalcon;
    private final CANCoder angleEncoder;
    private final double angleEncoderOffset;

    SimpleMotorFeedforward ffDriveController;

    private double lastAngle;

    public SwerveModule(int moduleId, DrivetrainConfig.ModuleConfig config) {
        this.moduleId = moduleId;


        driveFalcon = new TalonFX(config.driveId, Constants.kCanivoreId);
        configDriveFalcon(config.driveInvert);

        angleFalcon = new TalonFX(config.angleId, Constants.kCanivoreId);
        configAngleFalcon(config.angleInvert);

        angleEncoder = new CANCoder(config.encoderId, Constants.kCanivoreId);
        configEncoder(config.encoderInvert);

        ffDriveController = new SimpleMotorFeedforward(
                DrivetrainConfig.kDriveS,
                DrivetrainConfig.kDriveV,
                DrivetrainConfig.kDriveA
        );

        angleEncoderOffset = config.encoderOffset;

        lastAngle = getState().angle.getDegrees();
    }

    private void configEncoder(boolean inverted) {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.canCoderConfig);
        angleEncoder.configSensorDirection(inverted);
    }

    // TODO: determine if CANCoder is actually being used
    private void configAngleFalcon(boolean inverted) {
        angleFalcon.configFactoryDefault();
        angleFalcon.configAllSettings(Robot.ctreConfigs.angleMotorConfig);
        angleFalcon.setInverted(inverted);
        angleFalcon.setNeutralMode(SwerveDrivetrainConstants.ANGLE_NEUTRAL_MODE);
        angleFalcon.selectProfileSlot(0, 0);
        resetToAbsolute();
    }

    private void configDriveFalcon(boolean inverted) {
        driveFalcon.configFactoryDefault();
        driveFalcon.configAllSettings(Robot.ctreConfigs.driveMotorConfig);
        driveFalcon.setInverted(inverted);
        driveFalcon.setNeutralMode(SwerveDrivetrainConstants.DRIVE_NEUTRAL_MODE);
        driveFalcon.setSelectedSensorPosition(0);
    }

    public int getId() {
        return moduleId;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if (openLoop) {
            driveFalcon.set(
                    ControlMode.PercentOutput,
                    desiredState.speedMetersPerSecond / SwerveDrivetrainConstants.MAX_SPEED
            );
        } else {
            final var setpoint = Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond,
                    SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE,
                    SwerveDrivetrainConstants.DRIVE_GEAR_RATIO
            );
            final var ffOutput = ffDriveController.calculate(desiredState.speedMetersPerSecond);

            driveFalcon.set(ControlMode.Velocity, setpoint, DemandType.ArbitraryFeedForward, ffOutput);
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveDrivetrainConstants.MAX_SPEED * 0.01))
                ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%.
        // Prevents Jittering.

        angleFalcon.set(ControlMode.Position, Conversions.degreesToFalcon(angle,
                SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));

        lastAngle = angle;
    }


    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveFalcon.getSelectedSensorVelocity(),
                SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleFalcon.getSelectedSensorPosition()
                , SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    private void resetToAbsolute() {
        angleFalcon.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleEncoderOffset,
                SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleFalcon.getSelectedSensorPosition(),
                SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
    }

    public double getDriveEncoder() {
        return driveFalcon.getSelectedSensorPosition();
    }

    public double getDriveEncoderDegrees() {
        return angleEncoder.getAbsolutePosition();
    }

    public double getDriveVelocity() {
        return driveFalcon.getSelectedSensorVelocity();
    }


    public void zeroModule() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleEncoderOffset,
                -SwerveDrivetrainConstants.ANGLE_GEAR_RATIO);
        angleFalcon.setSelectedSensorPosition(absolutePosition);
    }


    public void moduleFortyFive() {
        angleFalcon.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder().getDegrees() - 45,
                -SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
    }

    public void moduleNegFortyFive() {
        angleFalcon.setSelectedSensorPosition(0);
        angleFalcon.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder().getDegrees() + 45,
                -SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
    }

    public double getDriveTemperature() {
        return driveFalcon.getTemperature();
    }

    public double getAngleTemperature() {
        return angleFalcon.getTemperature();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(Conversions.falconToMeters(
                driveFalcon.getSelectedSensorPosition(),
                SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO),
                getAngle()
        );
    }
}
