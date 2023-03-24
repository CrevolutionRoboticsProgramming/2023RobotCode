// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.drivetrainOld.SwerveDrivetrain;

public class AutoBalancing extends CommandBase {
  /** Creates a new AutoBalancing. */
  private final SwerveDrivetrain mSwerveDrivetrain;
  private final PIDController balanceController;

  private final double BALANCE_GOAL = 0;

  private final double BALANCE_kP = 0.05;
  private final double BALANCE_kI = 0.0;
  private final double BALANCE_kD = 0.01;

  private final double MAX_OUTPUT = 0.25 * SwerveDrivetrainConstants.MAX_SPEED;

  private double currentAngle, output; 

  public AutoBalancing(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerveDrivetrain = drivetrain;
    addRequirements(mSwerveDrivetrain);

    balanceController = new PIDController(BALANCE_kP, BALANCE_kI, BALANCE_kD);
    balanceController.setTolerance(1.0);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balanceController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = mSwerveDrivetrain.getRoll().getDegrees();

    output = MathUtil.clamp(balanceController.calculate(currentAngle, BALANCE_GOAL), -MAX_OUTPUT, MAX_OUTPUT);

    mSwerveDrivetrain.drive(new Translation2d(-output, 0), 0, true, true);

    System.out.println("Current Angle " + currentAngle + " Output " + output); 
    Logger.getInstance().recordOutput("AutoBalance/Current Angle", currentAngle);
    Logger.getInstance().recordOutput("AutoBalance/Current Output Speed", output);
    
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerveDrivetrain.stopSwerve();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(balanceController.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}
