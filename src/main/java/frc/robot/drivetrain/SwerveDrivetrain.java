package frc.robot.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.CrevoLib.math.Conversions;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveDriveOdometry odometry;
    public SwerveModule[] modules;
    public Pigeon2 pigeon;

    public SwerveModuleState[] currentModuleStates = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
    };

    public SwerveDrivetrain() {
        pigeon = new Pigeon2(SwerveDrivetrainConstants.PIGEON_ID);
        pigeon.configFactoryDefault();
        zeroGyro();

        modules = new SwerveModule[]{
                new SwerveModule(0,
                        SwerveDrivetrainConstants.FRONT_LEFT_OFFSET,
                        SwerveDrivetrainConstants.FRONT_LEFT_ANGLE_ID,
                        SwerveDrivetrainConstants.FRONT_LEFT_DRIVE_ID,
                        SwerveDrivetrainConstants.FRONT_LEFT_CANCODER_ID,
                        SwerveDrivetrainConstants.FRONT_LEFT_ANGLE_INVERT,
                        SwerveDrivetrainConstants.FRONT_LEFT_DRIVE_INVERT,
                        SwerveDrivetrainConstants.FRONT_LEFT_CANCODER_INVERT
                ),
                new SwerveModule(1,
                        SwerveDrivetrainConstants.FRONT_RIGHT_OFFSET,
                        SwerveDrivetrainConstants.FRONT_RIGHT_ANGLE_ID,
                        SwerveDrivetrainConstants.FRONT_RIGHT_DRIVE_ID,
                        SwerveDrivetrainConstants.FRONT_RIGHT_CANCODER_ID,
                        SwerveDrivetrainConstants.FRONT_RIGHT_ANGLE_INVERT,
                        SwerveDrivetrainConstants.FRONT_RIGHT_DRIVE_INVERT,
                        SwerveDrivetrainConstants.FRONT_RIGHT_CANCODER_INVERT
                ),
                new SwerveModule(2,
                        SwerveDrivetrainConstants.BACK_LEFT_OFFSET,
                        SwerveDrivetrainConstants.BACK_LEFT_ANGLE_ID,
                        SwerveDrivetrainConstants.BACK_LEFT_DRIVE_ID,
                        SwerveDrivetrainConstants.BACK_LEFT_CANCODER_ID,
                        SwerveDrivetrainConstants.BACK_LEFT_ANGLE_INVERT,
                        SwerveDrivetrainConstants.BACK_LEFT_DRIVE_INVERT,
                        SwerveDrivetrainConstants.BACK_LEFT_CANCODER_INVERT
                ),
                new SwerveModule(3,
                        SwerveDrivetrainConstants.BACK_RIGHT_OFFSET,
                        SwerveDrivetrainConstants.BACK_RIGHT_ANGLE_ID,
                        SwerveDrivetrainConstants.BACK_RIGHT_DRIVE_ID,
                        SwerveDrivetrainConstants.BACK_RIGHT_CANCODER_ID,
                        SwerveDrivetrainConstants.BACK_RIGHT_ANGLE_INVERT,
                        SwerveDrivetrainConstants.BACK_RIGHT_DRIVE_INVERT,
                        SwerveDrivetrainConstants.BACK_RIGHT_CANCODER_INVERT
                ),
        };

        odometry = new SwerveDriveOdometry(
                SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS,
                getYaw(),
                getModulePositions()
        );
    }

    public void drive(Translation2d translation, Rotation2d rotation, Translation2d rotationOffset,
                      boolean isFieldRelative, boolean isOpenLoop) {
        final ChassisSpeeds speeds;
        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation.getRadians(),
                    getYaw()
            );
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation.getRadians());
        }

        final var newStates = SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                speeds,
                rotationOffset
        );

        setModuleStates(newStates);

        // TODO: Move to setModuleStates?
        currentModuleStates = newStates;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDrivetrainConstants.MAX_SPEED);
        for (var mod : modules) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds));
    }

    public void stopSwerve() {
        drive(new Translation2d(0, 0), Rotation2d.fromRadians(0), new Translation2d(0, 0), true, true);
    }

    public ChassisSpeeds getChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond,
                                          double omegaRadiansPerSecond, Rotation2d robotAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                robotAngle
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getAngle() {
        return pigeon.getYaw();
    }

    public double getNonContinuousGyro() {
        return getAngle() % 360;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (var mod : modules) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        pigeon.setYaw(0);
    }

    public void zeroModules() {
        for (var mod : modules) {
            mod.zeroModule();
        }
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Rotation2d getYaw() {
        if (SwerveDrivetrainConstants.PIGEON_INVERT) {
            return Rotation2d.fromDegrees(360 - pigeon.getYaw());
        } else {
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(pigeon.getRoll());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        //UPDATE ODOMETRY
        odometry.update(getYaw(), getModulePositions());

        for (SwerveModule mod : modules) {
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        //UPDATE LOGGER (PERIODIC) -> AdvantageScope Configuration
        Logger.getInstance().recordOutput("Drive/Odometry/RobotPose2d", odometry.getPoseMeters());
        Logger.getInstance().recordOutput("Drive/Odometry/RobotPose3d", new Pose3d(odometry.getPoseMeters()));
        Logger.getInstance().recordOutput("Drive/Yaw/Robot", getNonContinuousGyro());


        Logger.getInstance().recordOutput("Drive/FLDrivePosition", modules[0].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/FLDriveVelocity",
                Conversions.falconToMPS(modules[0].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE,
                        SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/FLDriveTemperature", modules[0].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/FLAngleTemperature", modules[0].getAngleTemperature());

        Logger.getInstance().recordOutput("Drive/FRDrivePosition", modules[1].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/FRDriveVelocity",
                Conversions.falconToMPS(modules[1].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE,
                        SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/FRDriveTemperature", modules[1].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/FRAngleTemperature", modules[1].getAngleTemperature());

        Logger.getInstance().recordOutput("Drive/BLDrivePosition", modules[2].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/BLDriveVelocity",
                Conversions.falconToMPS(modules[2].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE,
                        SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/BLDriveTemperature", modules[2].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/BLAngleTemperature", modules[2].getAngleTemperature());

        Logger.getInstance().recordOutput("Drive/BRDrivePosition", modules[3].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/BRDriveVelocity",
                Conversions.falconToMPS(modules[3].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE,
                        SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/BRDriveTemperature", modules[3].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/BRAngleTemperature", modules[3].getAngleTemperature());

        Logger.getInstance().recordOutput("Drive/RealStates", getStates());
        Logger.getInstance().recordOutput("Drive/SetpointStates", currentModuleStates);

    }
}
