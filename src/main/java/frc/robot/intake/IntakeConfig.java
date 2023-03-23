package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.CrevoLib.math.Conversions;

public class IntakeConfig {
    public enum HoodState {
        kOpen(DoubleSolenoid.Value.kReverse), kClosed(DoubleSolenoid.Value.kForward);

        HoodState(DoubleSolenoid.Value state) {
            this.state = state;
        }

        public final DoubleSolenoid.Value state;
    }

    public enum PivotState {
        kDeployed(0),
        kStowed(Conversions.degreesToRadians(135)),
        kScoreLow(Conversions.degreesToRadians(60)),
        kHumanPlayer(Conversions.degreesToRadians(135)),
        kUnspecified(0);

        /**
         * Creates a PivotState with a target in radians
         *
         * @param target angle in radians
         */
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
    public static final int kLimitSwitchID = 7;

    public static final boolean kPivotMotorInverted = false;
    public static final boolean kRollerMotorInverted = false;

    public static final CANSparkMax.IdleMode kPivotIdleMode = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode kRollerIdleMode = CANSparkMax.IdleMode.kCoast;

    // Intake Profiles
    public static IntakeProfile kConeProfile = new IntakeProfile(20, 40, 1, "Cone");
    public static IntakeProfile kCubeProfile = new IntakeProfile(5, 30, 1, "Cube");
    public static IntakeProfile kOuttake = new IntakeProfile(40, 40, -1, "Output");
    public static IntakeProfile kDefaultProfile = new IntakeProfile(40, 40, 1, "Default");

    // Pivot PID constants
    public static final double kVelP = 0.0;
    public static final double kVelI = 0.0;
    public static final double kVelD = 0.0;

    // Pivot Feedforward Constants (reca.lc)
    public static final double kG = 0.23;
    public static final double kS = 0.0;
    public static final double kV = 2.92;
    public static final double kA = 0.01;

    // Motion Profile Parameters
    public static final double kMaxAngularAcceleration = 4.5;
    public static final double kMaxAngularVelocity = 3.0;

    // Sensor Parameters
    public static final boolean kPivotEncoderInverted = true;
    public static final double kPivotZeroOffset = 0.98;

    // Tensioning constants
    public static final double kTensionOutput = -0.03;
    public static final double kTensionFindOutput = -0.05;

    public static final int kDefaultContinuousCurrentLimit = 35;
    public static final int kDefaultPeakCurrentLimit = 60;

    public static final int kTensionContinuousCurrentLimit = 1;
    public static final int kTensionPeakCurrentLimit = 1;
}
