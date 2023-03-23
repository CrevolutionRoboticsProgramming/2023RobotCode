// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.claw.ClawConfig.ClawState;
import frc.robot.intake.IntakeConfig.HoodState;

public class Claw extends SubsystemBase {
    private final DoubleSolenoid clawSolenoid;
    private ClawState clawState;

    public Claw() {
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                ClawConfig.kClawForwardChannel,
                ClawConfig.kClawReverseChannel);

        clawState = ClawConfig.ClawState.kOpen;
    }


    public void setClawState(ClawConfig.ClawState state) {
        clawState = state;
        clawSolenoid.set(state.state);
    }

    public void toggleClawState() {
        // switch (clawState) {
        //     case kOpen:
        //         clawState = ClawConfig.ClawState.kClosed;
        //         break;
        //     case kClosed:
        //         clawState = ClawConfig.ClawState.kOpen;
        //         break;
        // }

        Value toggleValue = clawSolenoid.get() == Value.kForward ? Value.kReverse : Value.kForward;
        clawSolenoid.set(toggleValue);


        // System.out.println("Setting claw state: " + clawState.name());


    }
}
