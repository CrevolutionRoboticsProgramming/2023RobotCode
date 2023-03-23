// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;

public class RunElevatorManual extends CommandBase {

    private final DoubleSupplier supplier;
    private final Elevator elevator;

    public RunElevatorManual(Elevator elevator, DoubleSupplier supplier) {
        this.elevator = elevator;
        this.supplier = supplier;

        addRequirements(elevator);
    }


    @Override
    public void initialize() {
        elevator.setState(ElevatorConfig.ElevatorState.kUnspecified);
    }

    @Override
    public void execute() {
        final var output = supplier.getAsDouble();
        final var limitStates = elevator.getLimitStates();
        if (output < 0 && limitStates[0]) {
            elevator.setElevatorOutput(0);
        } else if (output > 0 && limitStates[1]) {
            elevator.setElevatorOutput(0);
        } else {
            elevator.setElevatorOutput(output * 12.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorOutput(0);
    }
}
