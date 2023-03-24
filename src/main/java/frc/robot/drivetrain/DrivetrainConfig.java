package frc.robot.drivetrain;

public class DrivetrainConfig {
    public static class ModuleConfig {
        private ModuleConfig(int driveId, int angleId, int encoderId, double encoderOffset, boolean driveInvert,
                             boolean angleInvert, boolean encoderInvert) {
            this.driveId = driveId;
            this.angleId = angleId;
            this.encoderId = encoderId;
            this.encoderOffset = encoderOffset;
            this.driveInvert = driveInvert;
            this.angleInvert = angleInvert;
            this.encoderInvert = encoderInvert;
        }

        public final int driveId;
        public final int angleId;
        public final int encoderId;
        public final double encoderOffset;
        public final boolean driveInvert;
        public final boolean angleInvert;
        public final boolean encoderInvert;
    }

    public static double kSlowModeTranslationModifier = 0.5;
    public static double kSlowModeRotationModifier = 0.5;

    public static final ModuleConfig kFrontLeftModuleConfig = new ModuleConfig(1, 3, 2, 176.66, true, false, false);
    public static final ModuleConfig kFrontRightModuleConfig = new ModuleConfig(10, 12, 11, 206.99, true, false, false);
    public static final ModuleConfig kBackLeftModuleConfig = new ModuleConfig(4, 6, 5, 100.98, true, false, false);
    public static final ModuleConfig kBackRightModuleConfig = new ModuleConfig(7, 9, 8, 67.99, true, false, false);

    // Feedback / Feedforward Constants
    public static final double kDriveS = (0.48665 / 12.0);
    public static final double kDriveV = (2.4132 / 12.0);
    public static final double kDriveA = (0.06921 / 12.0);
}
