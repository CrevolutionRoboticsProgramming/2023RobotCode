package frc.robot.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeConfig {
    public enum HoodState {
        kOpen(DoubleSolenoid.Value.kForward), kClosed(DoubleSolenoid.Value.kReverse);

        HoodState(DoubleSolenoid.Value state) {
            this.state = state;
        }

        public final DoubleSolenoid.Value state;
    }

    public enum PivotState {
        kDeployed(0), kStowed(Math.PI / 4), kExtract(0);

        PivotState(double target) {
            this.target = target;
        }

        public final double target;
    }

    public static class IntakeProfile {
        private IntakeProfile(int stallLimit, int freeLimit, double nominalSpeed, String name) {
            kStallCurrentLimit = stallLimit;
            kFreeCurrentLimit = freeLimit;
            kNominalOutput = nominalSpeed;
            kName = name;
        }

        @Override
        public String toString() {
            return kName;
        }

        final int kStallCurrentLimit;
        final int kFreeCurrentLimit;
        final double kNominalOutput;
        private final String kName;
    }

    // Devices
    public static final int kPivotSparkID = 28;
    public static final int kRollerSparkID = 46;
    public static final int kHoodForwardChannel = 0;
    public static final int kHoodReverseChannel = 1;

    public static final boolean kPivotMotorInverted = false;
    public static final boolean kRollerMotorInverted = false;

    // Intake Profiles
    public static IntakeProfile kConeProfile = new IntakeProfile(0, 0, 0.4, "Cone");
    public static IntakeProfile kCubeProfile = new IntakeProfile(0, 0, 0.9, "Cube");
    public static IntakeProfile kDefaultProfile = new IntakeProfile(0, 0, 1, "Default");

    // Pivot PID constants
    public static final double kVelP = 0.0;
    public static final double kVelI = 0.0;
    public static final double kVelD = 0.0;

    // Pivot Feedforward Constants
    public static final double kG = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    // Motion Profile Parameters
    public static final double kMaxAngularAcceleration = 0.0;
    public static final double kMaxAngularVelocity = 0.0;

    // Sensor Parameters
    public static final boolean kPivotEncoderInverted = true;
    public static final double kPivotZeroOffset = -0.116447;
}
