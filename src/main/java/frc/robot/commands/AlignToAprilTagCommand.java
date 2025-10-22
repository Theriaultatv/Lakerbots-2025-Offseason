package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final SwerveRequest.FieldCentric driveRequest;
    private final double lateralOffset; // Offset in meters (positive = left, negative = right)
    
    // PID-like constants for alignment
    private static final double YAW_KP = 0.05; // Proportional gain for rotation
    private static final double DISTANCE_KP = 1.5; // Proportional gain for forward/backward movement
    private static final double STRAFE_KP = 0.05; // Proportional gain for left/right movement
    
    // Target distance in meters
    private static final double TARGET_DISTANCE = 6.0;
    
    // Tolerance values
    private static final double YAW_TOLERANCE = 5.0; // degrees
    private static final double DISTANCE_TOLERANCE = 1.0; // meters
    private static final double LATERAL_TOLERANCE = 0.1; // meters
    
    // Speed limits
    private static final double MAX_SPEED = 1.0; // meters per second
    private static final double MAX_ROTATION_SPEED = 1.0; // radians per second
    
    private boolean isAligned = false;

    /**
     * Creates an AlignToAprilTagCommand with no lateral offset (center alignment)
     */
    public AlignToAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
        this(drivetrain, visionSubsystem, 0.0);
    }

    /**
     * Creates an AlignToAprilTagCommand with a lateral offset
     * @param drivetrain The swerve drivetrain
     * @param visionSubsystem The vision subsystem
     * @param lateralOffset Offset in meters (positive = left of tag, negative = right of tag)
     */
    public AlignToAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem, double lateralOffset) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.driveRequest = new SwerveRequest.FieldCentric();
        this.lateralOffset = lateralOffset;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isAligned = false;
        SmartDashboard.putBoolean("Auto Align Active", true);
    }

    @Override
    public void execute() {
        // Try FL camera first, then FR camera
        boolean targetVisible = visionSubsystem.isTargetVisibleFL();
        double targetYaw = visionSubsystem.getTargetYawFL();
        double targetDistance = visionSubsystem.getTargetDistanceFL();
        int detectedTagId = visionSubsystem.getDetectedTagIdFL();
        
        // If FL doesn't see target, try FR
        if (!targetVisible) {
            targetVisible = visionSubsystem.isTargetVisibleFR();
            targetYaw = visionSubsystem.getTargetYawFR();
            targetDistance = visionSubsystem.getTargetDistanceFR();
            detectedTagId = visionSubsystem.getDetectedTagIdFR();
        }
        
        if (!targetVisible) {
            // No target visible, stop the robot
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            SmartDashboard.putString("Auto Align Status", "No Target Visible");
            return;
        }
        
        // Calculate error values
        double yawError = targetYaw; // Positive yaw means target is to the right
        double distanceError = targetDistance - TARGET_DISTANCE; // Positive means too far, negative means too close
        
        // Calculate lateral error based on offset
        // When lateralOffset is positive (want to be left of tag), and yaw is 0 (centered),
        // we need to strafe left (positive Y velocity in field-centric)
        double lateralError = -lateralOffset; // Negative because we want to move opposite to achieve offset
        
        // Calculate control outputs
        double rotationSpeedRaw = -yawError * YAW_KP; // Negative to turn toward target
        double forwardSpeedRaw = distanceError * DISTANCE_KP; // Positive error (too far) = positive speed (move forward)
        double strafeSpeedRaw = lateralError * STRAFE_KP; // Strafe to achieve lateral offset
        
        // Limit speeds
        double rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeedRaw));
        double forwardSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, forwardSpeedRaw));
        double strafeSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, strafeSpeedRaw));
        
        // Check if aligned (including lateral position if offset is specified)
        boolean lateralAligned = Math.abs(lateralOffset) < 0.01 || Math.abs(lateralError) < LATERAL_TOLERANCE;
        isAligned = Math.abs(yawError) < YAW_TOLERANCE && Math.abs(distanceError) < DISTANCE_TOLERANCE && lateralAligned;
        
        // If aligned, stop the robot. Otherwise, apply drive commands
        if (isAligned) {
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        } else {
            // Apply drive request using setControl
            drivetrain.setControl(driveRequest
                .withVelocityX(forwardSpeed) // Forward/backward
                .withVelocityY(strafeSpeed) // Left/right strafing for offset
                .withRotationalRate(rotationSpeed)); // Rotation
        }
        
        // Update SmartDashboard
        SmartDashboard.putString("Auto Align Status", isAligned ? "ALIGNED" : "Aligning...");
        SmartDashboard.putNumber("Auto Align Yaw Error", yawError);
        SmartDashboard.putNumber("Auto Align Distance Error", distanceError);
        SmartDashboard.putNumber("Auto Align Lateral Error", lateralError);
        SmartDashboard.putNumber("Auto Align Lateral Offset", lateralOffset);
        SmartDashboard.putNumber("Auto Align Target Distance", targetDistance);
        SmartDashboard.putNumber("Auto Align Detected Tag", detectedTagId);
        SmartDashboard.putNumber("Auto Align Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("Auto Align Strafe Speed", strafeSpeed);
        SmartDashboard.putNumber("Auto Align Rotation Speed", rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        
        SmartDashboard.putBoolean("Auto Align Active", false);
        SmartDashboard.putString("Auto Align Status", interrupted ? "Interrupted" : "Complete");
    }

    @Override
    public boolean isFinished() {
        // Command continues until button is released or target is lost for too long
        return false; // Let the button release end the command
    }
}
