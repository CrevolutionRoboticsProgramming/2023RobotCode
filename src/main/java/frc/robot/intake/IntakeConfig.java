package frc.robot.intake;

public class IntakeConfig {
    public enum HoodState {
        kOpen, kClosed,
    }

    public enum PivotState {
        kDeployed(0), kStowed(0), kExtract(0);

        private PivotState(int target) {
            this.target = target;
        }

        final int target;
    }

    // Devices
    static final int kPivotSparkID = 28;
    static final int kRollerSparkID = 51;
    static final int kHoodForwardChannel = 0;
    static final int kHoodReverseChannel = 1;

    // Constants
    static class IntakeProfile {
        private IntakeProfile(int stallLimit, int freeLimit, double nominalSpeed) {
            kStallCurrentLimit = stallLimit;
            kFreeCurrentLimit = freeLimit;
            kNominalOutput = nominalSpeed;
        }

        final int kStallCurrentLimit;
        final int kFreeCurrentLimit;
        final double kNominalOutput;
    }

    static IntakeProfile kConeProfile = new IntakeProfile(0, 0, 1);
    static IntakeProfile kCubeProfile = new IntakeProfile(0, 0, 1);
    static IntakeProfile kDefaultProfile = new IntakeProfile(0, 0, 1);

    // Pivot PID constants
    static final double kPivotP = 0.0;
    static final double kPivotI = 0.0;
    static final double kPivotD = 0.0;
    static final double kPivotF = 0.0;
}
