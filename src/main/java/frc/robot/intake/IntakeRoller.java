package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
    private final CANSparkMax rollerSpark;
    private double nominalOutput = 1.0;

    public IntakeRoller() {
        rollerSpark = new CANSparkMax(IntakeConfig.kRollerSparkID, CANSparkMaxLowLevel.MotorType.kBrushless);
        setProfile(IntakeConfig.kDefaultProfile);
    }

    public void setOutput(double output) {
        final var adjusted_output = output * nominalOutput;
        rollerSpark.set(adjusted_output);
    }

    public void stop() {
        rollerSpark.set(0);
    }

    public void setProfile(IntakeConfig.IntakeProfile profile) {
        rollerSpark.setSmartCurrentLimit(profile.kStallCurrentLimit, profile.kFreeCurrentLimit);
        nominalOutput = profile.kNominalOutput;
    }
}
