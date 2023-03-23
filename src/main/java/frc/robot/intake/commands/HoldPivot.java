package frc.robot.intake.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakePivot;

public class HoldPivot extends CommandBase {
    private final IntakePivot pivot;
    private IntakeConfig.PivotState state;

    public HoldPivot(IntakePivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        state = pivot.getState();
        switch (state) {
            case kDeployed:
                pivot.setCurrentLimit(
                        IntakeConfig.kTensionContinuousCurrentLimit,
                        IntakeConfig.kTensionPeakCurrentLimit
                );
                pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
                break;
            case kStowed:
                pivot.setIdleMode(CANSparkMax.IdleMode.kCoast);
                break;
            default:
                pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
                break;
        }
        System.out.println("HoldState[" + state.name() + "]");
    }

    @Override
    public void execute() {
        if (state == IntakeConfig.PivotState.kDeployed) {
            pivot.setOutput(pivot.getLimitSwitchState() ? IntakeConfig.kTensionOutput : IntakeConfig.kTensionFindOutput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setCurrentLimit(IntakeConfig.kDefaultContinuousCurrentLimit, IntakeConfig.kDefaultPeakCurrentLimit);
        pivot.stop();
    }
}
