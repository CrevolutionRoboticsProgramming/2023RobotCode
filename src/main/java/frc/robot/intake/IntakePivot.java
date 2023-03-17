package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CrevoLib.math.Conversions;

public class IntakePivot extends SubsystemBase {
    private final CANSparkMax spark;
    private final SparkMaxAbsoluteEncoder encoder;
    private final DoubleSolenoid hoodSolenoid;

    private IntakeConfig.HoodState hoodState;

    public IntakePivot() {
        spark = new CANSparkMax(IntakeConfig.kPivotSparkID, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = spark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        hoodSolenoid = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                IntakeConfig.kHoodForwardChannel,
                IntakeConfig.kHoodReverseChannel
        );

        spark.setIdleMode(CANSparkMax.IdleMode.kBrake);

        hoodState = IntakeConfig.HoodState.kClosed;

        configureMotor();
        configureSensors();
    }

    private void configureMotor() {
        spark.setInverted(IntakeConfig.kPivotMotorInverted);
        spark.setSmartCurrentLimit(40, 40);
    }

    private void configureSensors() {
        encoder.setZeroOffset(IntakeConfig.kPivotZeroOffset);
        encoder.setInverted(IntakeConfig.kPivotEncoderInverted);
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
        return Conversions.rotationToRadians(encoder.getVelocity());
    }

    public double getOutputCurrent() {
        return Math.abs(spark.getOutputCurrent());
    }

    /**
     * Sets the voltage of the pivot motor
     * @param voltage voltage
     */
    public void set(double voltage) {
        spark.set(voltage);
    }

    public void stop() {
        spark.set(0);
    }

    public void setHoodState(IntakeConfig.HoodState state) {
        hoodState = state;
        hoodSolenoid.set(state.state);
    }

    public void toggleHoodState() {
        switch (hoodState) {
            case kOpen:
                hoodState = IntakeConfig.HoodState.kClosed;
                break;
            case kClosed:
                hoodState = IntakeConfig.HoodState.kOpen;
                break;
        }

        System.out.println("SetIntakeState[" + hoodState.name() + "]");

        hoodSolenoid.set(hoodState.state);
    }
}
