package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax pivotSpark, rollerSpark;
    private final DoubleSolenoid hoodSolenoid;

    private double nominalOutput;

    private IntakeConfig.PivotState pivotState;
    private PIDController pivotController;

    public Intake() {
        pivotSpark = new CANSparkMax(IntakeConfig.kPivotSparkID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerSpark = new CANSparkMax(IntakeConfig.kRollerSparkID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hoodSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConfig.kHoodForwardChannel, IntakeConfig.kHoodReverseChannel);

        rollerSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivotController = new PIDController(IntakeConfig.kPivotP, IntakeConfig.kPivotI, IntakeConfig.kPivotD, IntakeConfig.kPivotF);

        applyRollerProfile(IntakeConfig.kDefaultProfile);
    }

    @Override
    public void periodic() {
//        updatePivotOutput(pivotState);
    }

    public void setRollerOutput(double output) {
        final var adjusted_output = output * (1.0 / nominalOutput);
        rollerSpark.set(adjusted_output);
    }

    public void applyRollerProfile(IntakeConfig.IntakeProfile profile) {
        rollerSpark.setSmartCurrentLimit(profile.kStallCurrentLimit, profile.kFreeCurrentLimit);
        nominalOutput = profile.kNominalOutput;
    }

    public void setPivotState(IntakeConfig.PivotState state) {
        pivotState = state;
    }

    @Deprecated
    public void setPivotOutput(double output) {
        pivotSpark.set(output);
    }

    private void updatePivotOutput(IntakeConfig.PivotState state) {
        final var position = pivotSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        pivotController.calculate(position, state.target);
    }

    public void setHoodState(IntakeConfig.HoodState state) {
        if (state == IntakeConfig.HoodState.kOpen) {
            hoodSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            hoodSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }
}
