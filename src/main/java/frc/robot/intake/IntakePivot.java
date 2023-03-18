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

    private IntakeConfig.HoodState hoodState;

    public IntakePivot() {
        spark = new CANSparkMax(IntakeConfig.kPivotSparkID, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = spark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        configureMotor();
        configureSensors();
    }

    private void configureMotor() {
        spark.setInverted(IntakeConfig.kPivotMotorInverted);
        spark.setSmartCurrentLimit(40, 40);
        spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
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
     * Sets the output of the pivot motor (range -1 to 1)
     * @param output output
     */
    public void set(double output) {
        spark.set(output);
    }

    public void stop() {
        spark.set(0);
    }
}
