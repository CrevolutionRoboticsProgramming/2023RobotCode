// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;

public class RunElevatorManual extends CommandBase {
  
  private final DoubleSupplier supplier;
  private final Elevator elevator;

  public RunElevatorManual(Elevator elevator, DoubleSupplier supplier) {
    this.elevator = elevator;
    this.supplier = supplier;
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.setElevatorOuput(supplier.getAsDouble());
  }


  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorOuput(0);
  }
 
  @Override
  public boolean isFinished() {
    return false;
  }
}
