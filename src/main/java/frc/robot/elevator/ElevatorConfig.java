package frc.robot.elevator;

public class ElevatorConfig {
    public enum ElevatorPosition {
        kLowCone(5000), 
        kHighCone(8000), 
        kLowCube(5000), 
        kHighCube(8000);

        private final int targetPos;

        ElevatorPosition(int targetPos) {
            this.targetPos = targetPos;
        }

        public int getElevatorPos(){
            return this.targetPos;
        }

    }

    public static final int kElevatorSparkID = 51;
    public static final int smartMotionSlot = 0;
    //Elevator PID Constants
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0; 
    
    public static final double kIz = 0.0;
    public static final double kFF = 0.0;

    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;

    public static final double maxRPM = 5700;
    public static final double maxVel = 2000;
    public static final double minVel = 0;
    public static final double maxAcc = 1500;
    public static final double allowedErr = 0;

    public static final boolean kElevatorMotorInverted = false;

    public static final double kElevatorZeroOffset = 0.0;
    public static final boolean kElevatorEncoderInverted = false;
}

