package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.drivetrain.SwerveDrivetrain;

import java.util.function.Supplier;

public class TeleopDrive extends CommandBase {
    private final SwerveDrivetrain drivetrain;
    private final Translation2d rotationOffset;
    private final Supplier<Translation2d> translationSupplier;
    private final Supplier<Rotation2d> rotationSupplier;
    private final boolean isFieldRelative, isOpenLoop;

    public TeleopDrive(SwerveDrivetrain drivetrain, Supplier<Translation2d> translation,
                       Supplier<Rotation2d> rotation, boolean isFieldRelative, Translation2d rotationOffset) {
        this.drivetrain = drivetrain;
        this.rotationOffset = rotationOffset;
        this.translationSupplier = translation;
        this.rotationSupplier = rotation;
        this.isFieldRelative = isFieldRelative;
        this.isOpenLoop = false;

        addRequirements(this.drivetrain);
    }

    public TeleopDrive(SwerveDrivetrain drivetrain, Supplier<Translation2d> translation,
                       Supplier<Rotation2d> rotation, boolean isFieldRelative) {
        this(drivetrain, translation, rotation, isFieldRelative, new Translation2d(0, 0));
    }

    @Override
    public void execute() {
        final var translation = translationSupplier.get().times(SwerveDrivetrainConstants.MAX_SPEED);
        final var rotation = rotationSupplier.get().times(SwerveDrivetrainConstants.MAX_ANGULAR_VELOCITY);

        drivetrain.drive(translation, rotation, rotationOffset, isFieldRelative, isOpenLoop);
    }

}
