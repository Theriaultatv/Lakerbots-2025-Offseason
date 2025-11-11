// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PumpkinSubsystem;

/**
 * Command to run the Pumpkin motor at 2V for 4 seconds
 */
public class RunPumpkinCommand extends Command {
    private final PumpkinSubsystem pumpkinSubsystem;
    private final double voltage;
    private final double duration;
    
    /**
     * Creates a new RunPumpkinCommand with default values from Constants
     * @param pumpkinSubsystem The Pumpkin subsystem to use
     */
    public RunPumpkinCommand(PumpkinSubsystem pumpkinSubsystem) {
        this(pumpkinSubsystem, Constants.Pumpkin.kVoltageOutput, Constants.Pumpkin.kRunDuration);
    }
    
    /**
     * Creates a new RunPumpkinCommand with custom values
     * @param pumpkinSubsystem The Pumpkin subsystem to use
     * @param voltage The voltage to apply
     * @param duration The duration to run in seconds
     */
    public RunPumpkinCommand(PumpkinSubsystem pumpkinSubsystem, double voltage, double duration) {
        this.pumpkinSubsystem = pumpkinSubsystem;
        this.voltage = voltage;
        this.duration = duration;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(pumpkinSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("RunPumpkinCommand: Starting motor at " + voltage + "V for " + duration + " seconds");
        pumpkinSubsystem.setVoltage(voltage);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Motor continues running at set voltage
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("RunPumpkinCommand: Stopping motor" + (interrupted ? " (interrupted)" : ""));
        pumpkinSubsystem.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Command finishes after the specified duration
        return false; // Timeout will handle this
    }
    
    /**
     * Creates this command with a timeout
     * @return This command with timeout applied
     */
    public Command withTimeout() {
        return this.withTimeout(duration);
    }
}
