// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.elevator.ElevatorConfig.ElevatorPosition;
import frc.robot.elevator.ElevatorConfig;

public class Elevator extends SubsystemBase {
    private final CANSparkMax elevatorSpark;
    private final RelativeEncoder elevatorEncoder;
    private final SparkMaxPIDController elevatorPIDController;

    public Elevator() {
        elevatorSpark = new CANSparkMax(ElevatorConfig.kElevatorSparkID, MotorType.kBrushless);
        elevatorEncoder = elevatorSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        elevatorSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);

        configureElevatorMotor();
        configureElevatorSensors();
        elevatorSpark.restoreFactoryDefaults();

        elevatorPIDController = elevatorSpark.getPIDController();

        elevatorPIDController.setP(ElevatorConfig.kP);
        elevatorPIDController.setI(ElevatorConfig.kI);
        elevatorPIDController.setD(ElevatorConfig.kD);
        elevatorPIDController.setIZone(ElevatorConfig.kIz);
        elevatorPIDController.setFF(ElevatorConfig.kFF);
        elevatorPIDController.setOutputRange(ElevatorConfig.kMinOutput, ElevatorConfig.kMaxOutput);


        elevatorPIDController.setSmartMotionMaxVelocity(ElevatorConfig.maxVel, ElevatorConfig.smartMotionSlot);
        elevatorPIDController.setSmartMotionMinOutputVelocity(ElevatorConfig.minVel, ElevatorConfig.smartMotionSlot);
        elevatorPIDController.setSmartMotionMaxAccel(ElevatorConfig.maxAcc, ElevatorConfig.smartMotionSlot);
        elevatorPIDController.setSmartMotionAllowedClosedLoopError(ElevatorConfig.allowedErr,
                ElevatorConfig.smartMotionSlot);

    }

    @Override
    public void periodic() {
//        System.out.println("[Elevator] pos: " + elevatorEncoder.getPosition() + ", vel: " + elevatorEncoder.getVelocity());
    }

    private void configureElevatorMotor() {
        elevatorSpark.setInverted(ElevatorConfig.kElevatorMotorInverted);
    }

    public void setElevatorOuput(double output) {
//        System.out.println(output);
        elevatorSpark.set(output);
    }

    private void configureElevatorSensors() {
        elevatorEncoder.setInverted(ElevatorConfig.kElevatorEncoderInverted);
    }

    public void resetElevator() {
        elevatorPIDController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    }

    public double getElevatorPosition() {
        return elevatorEncoder.getPosition();
    }

    public void setElevatorSetpoint(ElevatorPosition pos) {
        elevatorPIDController.setReference(pos.getElevatorPos(), CANSparkMax.ControlType.kSmartMotion);
    }

    public void resetEncoder() {
        elevatorEncoder.setPosition(0);
    }


}
