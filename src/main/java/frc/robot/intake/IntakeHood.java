package frc.robot.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeHood extends SubsystemBase {
    private DoubleSolenoid solenoid;
    private IntakeConfig.HoodState state;

    public IntakeHood() {
        solenoid = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                IntakeConfig.kHoodForwardChannel,
                IntakeConfig.kHoodReverseChannel
        );

        state = IntakeConfig.HoodState.kClosed;
    }

    public IntakeConfig.HoodState getState() {
        return state;
    }

    public void setState(IntakeConfig.HoodState state) {
        this.state = state;
        solenoid.set(state.state);
    }
}
