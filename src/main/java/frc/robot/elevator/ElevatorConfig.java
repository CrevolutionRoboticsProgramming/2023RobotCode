package frc.robot.elevator;

import frc.robot.CrevoLib.math.Conversions;

public class ElevatorConfig {
    public enum ElevatorState {
        kZero(0),
        kZeroGoal(0),
        kMid(0.85),
        kHigh(1.15),
        kHighGoal(1.15),
        kLoad(0.0437),
        kChamber(0.0935),
        kPreScore(0.165),
        kUnspecified(0);
        
        public final double target;

        ElevatorState(double target) {
            this.target = target;
        }
    }

    // Devices
    public static final int kElevatorSparkID = 51;
    public static final int kUpperLimitSwitchPort = 9;
    public static final int kLowerLimitSwitchPort = 8;

    public static final boolean kElevatorMotorInverted = true;
    public static final boolean kElevatorEncoderInverted = false;

    // Elevator PID Constants
    public static final double kP = 4;
    public static final double kI = 0.0;
    public static final double kD = 0.0; 

    // Elevator FF constants (for rio profiling)
    public static final double kS = 0.0;
    public static final double kV = 21.5;
    public static final double kA = 1;
    public static final double kG = 0.4;

    // Max velocity and acceleration
    public static final double kMaxVelocity = 0.5;
    public static final double kMaxAcceleration = 0.6;

    public static final double kSprocketDiameter = Conversions.feetToMeters(1.273 / 12.0);

    public static final double kSeekVoltage = 2;
}

