package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CrevoLib.math.Conversions;

public class IntakePivot extends SubsystemBase {
    private final CANSparkMax spark;
    private final SparkMaxAbsoluteEncoder encoder;
    private final DigitalInput limitSwitch;

    private IntakeConfig.PivotState state;

    public IntakePivot() {
        spark = new CANSparkMax(IntakeConfig.kPivotSparkID, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = spark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        limitSwitch = new DigitalInput(IntakeConfig.kLimitSwitchID);

        state = IntakeConfig.PivotState.kUnspecified;

        configureMotor();
        configureSensors();
    }

    private void configureMotor() {
        spark.setInverted(IntakeConfig.kPivotMotorInverted);
        spark.setIdleMode(IntakeConfig.kPivotIdleMode);
        setCurrentLimit(IntakeConfig.kDefaultContinuousCurrentLimit, IntakeConfig.kDefaultPeakCurrentLimit);
    }

    private void configureSensors() {
        encoder.setZeroOffset(IntakeConfig.kPivotZeroOffset);
        encoder.setInverted(IntakeConfig.kPivotEncoderInverted);
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        spark.setIdleMode(mode);
    }

    /**
     * @return position in radians
     */
    public double getAngleRads() {
        final var angle = Conversions.rotationToRadians(encoder.getPosition());
        return (angle >= 6) ? 0 : angle;
    }

    /**
     * @return angular velocity in rads / sec
     */
    public double getVelocityRps() {
        return Conversions.rotationToRadians(encoder.getVelocity() / 60.0);
    }

    public double getOutputCurrent() {
        return Math.abs(spark.getOutputCurrent());
    }

    /**
     * Sets the output of the pivot motor (range -1 to 1)
     * @param output output
     */
    public void setOutput(double output) {
        spark.set(output);
    }

    public void setCurrentLimit(int continuousLimit, int peakLimit) {
        spark.setSmartCurrentLimit(continuousLimit, peakLimit);
    }

    public void stop() {
        spark.set(0);
    }

    public IntakeConfig.PivotState getState() {
        return state;
    }

    public void setState(IntakeConfig.PivotState state) {
        this.state = state;
    }

    public boolean getLimitSwitchState() {
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        System.out.println("[pivot] pos (" + getAngleRads() + "), vel (" + getVelocityRps() + ")");
    }
}
