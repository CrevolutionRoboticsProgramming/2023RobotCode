// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.elevator.ElevatorConfig.ElevatorState;

public class Elevator extends SubsystemBase {
    private final CANSparkMax spark;
    private final RelativeEncoder encoder;
    private final DigitalInput lowerLimitSwitch, upperLimitSwitch;

    private ElevatorState currentState;

    public Elevator() {
        // Setup devices
        spark = new CANSparkMax(ElevatorConfig.kElevatorSparkID, MotorType.kBrushless);
        encoder = spark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        lowerLimitSwitch = new DigitalInput(ElevatorConfig.kLowerLimitSwitchPort);
        upperLimitSwitch = new DigitalInput(ElevatorConfig.kUpperLimitSwitchPort);

        configMotors();
        configSensors();

        currentState = ElevatorState.kUnspecified;
    }

    private void configMotors() {
        spark.restoreFactoryDefaults();
        spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        spark.setInverted(ElevatorConfig.kElevatorMotorInverted);
    }

    private void configSensors() {
        encoder.setInverted(ElevatorConfig.kElevatorEncoderInverted);
    }

    public void zero() {
        encoder.setPosition(0);
    }

    public double getPositionMeters() {
        return ElevatorUtils.rotationsToMeters(encoder.getPosition());
    }

    public double getVelocityMps() {
        return ElevatorUtils.rotationsToMeters(encoder.getVelocity() / 60.0);
    }

    public boolean[] getLimitStates() {
        return new boolean[]{!lowerLimitSwitch.get(), !upperLimitSwitch.get()};
    }

    public void setElevatorOutput(double output) {
        final var switchStates = getLimitStates();
        if(switchStates[0] && output < 0) {
            spark.setVoltage(0);
        }
        else if(switchStates[1] && output > 0) {
            spark.setVoltage(0);
        }
        else {
            spark.setVoltage(output);
        }
    }

    public void stop() {
        spark.set(0);
    }

    public void setState(ElevatorState state) {
        currentState = state;
    }

    public ElevatorState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        final var states = getLimitStates();
//        System.out.println("[Elevator] pos: " + getPositionMeters() + ", vel: " + getVelocityMps());

        if (getLimitStates()[0]) {
            currentState = ElevatorState.kZero;
            zero();
        }
    }
}
