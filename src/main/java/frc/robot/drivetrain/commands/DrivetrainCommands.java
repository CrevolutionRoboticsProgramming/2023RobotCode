package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class DrivetrainCommands {
    public static Command driveFieldOriented(DoubleSupplier translationX, DoubleSupplier translationY,
                                             DoubleSupplier rotation) {
        return drive(translationX, translationY, rotation, true, new Translation2d(0, 0));
    }

    public static Command driveRobotOriented(DoubleSupplier translationX, DoubleSupplier translationY,
                                             DoubleSupplier rotation) {
        return drive(translationX, translationY, rotation, false, new Translation2d(0, 0));
    }

    public static Command driveWithIntakeRotation(DoubleSupplier translationX, DoubleSupplier translationY,
                                                  DoubleSupplier rotation) {
        return drive(translationX, translationY, rotation, true, new Translation2d(
                Constants.SwerveDrivetrainConstants.DRIVETRAIN_ACTUAL_LENGTH / 2.0,
                0
        ));
    }

    // TODO: Legacy bug where Y and X inputs are inverted in the Translation2d
    private static Command drive(DoubleSupplier translationX, DoubleSupplier translationY,
                                 DoubleSupplier rotation, boolean isFieldOriented, Translation2d rotationOffset) {
        return new TeleopDrive(
                RobotContainer.drivetrain,
                () -> new Translation2d(
                        translationY.getAsDouble(),
                        translationX.getAsDouble()
                ).times(Constants.SwerveDrivetrainConstants.MAX_SPEED),
                () -> Rotation2d.fromRadians(
                        rotation.getAsDouble()
                ).times(Constants.SwerveDrivetrainConstants.MAX_ANGULAR_VELOCITY),
                true
        );
    }
}
