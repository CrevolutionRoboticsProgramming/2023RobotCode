package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.CrevoLib.io.JoystickMods;
import frc.robot.drivetrain.SwerveDrivetrain;

import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
    private final SwerveDrivetrain drivetrain;
    private final DoubleSupplier txSupplier, tySupplier, rSupplier;
    private final boolean isFieldRelative;
    private final boolean isOpenLoop;

    public TeleopDrive(SwerveDrivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY,
                       DoubleSupplier rotation, boolean isFieldRelative, boolean isOpenLoop) {
        this.drivetrain = drivetrain;
        txSupplier = translationX;
        tySupplier = translationY;
        rSupplier = rotation;
        this.isFieldRelative = isFieldRelative;
        this.isOpenLoop = isOpenLoop;

        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        final var translation = new Translation2d(
                tySupplier.getAsDouble(),
                txSupplier.getAsDouble()
        ).times(SwerveDrivetrainConstants.MAX_SPEED);

        final var rotation = rSupplier.getAsDouble() * SwerveDrivetrainConstants.MAX_ANGULAR_VELOCITY;

        drivetrain.drive(translation, rotation, isFieldRelative, isOpenLoop);
    }

}
