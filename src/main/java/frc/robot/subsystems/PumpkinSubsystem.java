// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Pumpkin subsystem - controls a motor on CAN ID 13
 */
public class PumpkinSubsystem extends SubsystemBase {
    private final TalonFX pumpkinMotor;
    private final VoltageOut voltageRequest;
    
    /** Creates a new PumpkinSubsystem. */
    public PumpkinSubsystem() {
        // Initialize the TalonFX motor on CAN ID 13 on the CANivore bus
        pumpkinMotor = new TalonFX(Constants.Pumpkin.kMotorCANId, Constants.Pumpkin.kCANBusName);
        
        // Create a VoltageOut control request
        voltageRequest = new VoltageOut(0);
        
        // Configure motor to coast mode when neutral
        var motorConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
        pumpkinMotor.getConfigurator().apply(motorConfig);
    }
    
    /**
     * Sets the motor voltage
     * @param voltage The voltage to apply (-12 to 12 volts)
     */
    public void setVoltage(double voltage) {
        pumpkinMotor.setControl(voltageRequest.withOutput(voltage));
    }
    
    /**
     * Stops the motor
     */
    public void stop() {
        pumpkinMotor.setControl(voltageRequest.withOutput(0));
    }
    
    /**
     * Gets the current motor voltage
     * @return The motor voltage in volts
     */
    public double getMotorVoltage() {
        return pumpkinMotor.getMotorVoltage().getValueAsDouble();
    }
    
    /**
     * Gets the current motor current
     * @return The motor current in amps
     */
    public double getMotorCurrent() {
        return pumpkinMotor.getSupplyCurrent().getValueAsDouble();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update telemetry
        SmartDashboard.putNumber("Pumpkin Motor Voltage", getMotorVoltage());
        SmartDashboard.putNumber("Pumpkin Motor Current", getMotorCurrent());
    }
}
