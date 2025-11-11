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
    
    // PID-like constants for alignment (tuned to reduce oscillation)
    private static final double YAW_KP = 0.01; // Reduced from 0.03 - less aggressive rotation
    private static final double DISTANCE_KP = 0.25; // Reduced from 0.5 - smoother approach
    private static final double STRAFE_KP = 0.015; // Reduced from 0.04 - less aggressive strafe
    
    // Target distance in meters
    private static final double TARGET_DISTANCE = 7.0;
    
    // Tolerance values (wider to prevent jitter)
    private static final double YAW_TOLERANCE = 3.0; // Increased from 2.0 degrees
    private static final double DISTANCE_TOLERANCE = 0.3; // Increased from 0.25 meters
    private static final double LATERAL_TOLERANCE = 0.15; // Increased from 0.1 meters
    
    // Deadband - minimum error before applying correction (prevents jitter)
    private static final double YAW_DEADBAND = 4.0; // degrees
    private static final double DISTANCE_DEADBAND = 0.25; // meters
    private static final double LATERAL_DEADBAND = 0.05; // meters
    
    // Speed limits
    private static final double MAX_SPEED = 0.8; // Reduced from 1.0 - smoother movement
    private static final double MAX_ROTATION_SPEED = 0.8; // Reduced from 1.0 - smoother rotation
    private static final double MIN_SPEED = 0.05; // Minimum speed to overcome friction
    
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
        // Camera offset from robot center in meters (from Constants)
        final double CAMERA_OFFSET_Y = 0.24765; // 9.75 inches in meters
        
        // Try FL camera first, then FR camera
        boolean targetVisible = visionSubsystem.isTargetVisibleFL();
        double targetYaw = visionSubsystem.getTargetYawFL();
        double targetDistance = visionSubsystem.getTargetDistanceFL();
        int detectedTagId = visionSubsystem.getDetectedTagIdFL();
        double cameraOffsetY = CAMERA_OFFSET_Y; // FL camera is to the left (+Y)
        String cameraUsed = "FL";
        
        // If FL doesn't see target, try FR
        if (!targetVisible) {
            targetVisible = visionSubsystem.isTargetVisibleFR();
            targetYaw = visionSubsystem.getTargetYawFR();
            targetDistance = visionSubsystem.getTargetDistanceFR();
            detectedTagId = visionSubsystem.getDetectedTagIdFR();
            cameraOffsetY = -CAMERA_OFFSET_Y; // FR camera is to the right (-Y)
            cameraUsed = "FR";
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
        // Compensate for camera offset: when camera sees target at 0° yaw, 
        // the robot needs to rotate to point the CENTER at the target
        double yawError = targetYaw; // Positive yaw means target is to the right of camera
        double distanceError = targetDistance - TARGET_DISTANCE; // Positive means too far, negative means too close
        
        // Calculate the angle offset needed to center the robot on the target
        // This accounts for the camera being offset from robot center
        double angleOffsetDeg = Math.toDegrees(Math.atan2(cameraOffsetY, targetDistance));
        
        // Adjust yaw error to align robot center (not camera) with target
        double robotCenterYawError = yawError - angleOffsetDeg;
        
        // Calculate lateral error based on desired offset
        double lateralError = -lateralOffset;
        
        // Apply deadband to prevent jitter when close to target
        double yawErrorWithDeadband = Math.abs(robotCenterYawError) < YAW_DEADBAND ? 0 : robotCenterYawError;
        double distanceErrorWithDeadband = Math.abs(distanceError) < DISTANCE_DEADBAND ? 0 : distanceError;
        double lateralErrorWithDeadband = Math.abs(lateralError) < LATERAL_DEADBAND ? 0 : lateralError;
        
        // Calculate control outputs using robot-center-corrected yaw error with deadband
        double rotationSpeedRaw = -yawErrorWithDeadband * YAW_KP;
        double forwardSpeedRaw = distanceErrorWithDeadband * DISTANCE_KP;
        double strafeSpeedRaw = lateralErrorWithDeadband * STRAFE_KP;
        
        // Apply minimum speed threshold to overcome friction (only if not zero)
        if (Math.abs(rotationSpeedRaw) > 0 && Math.abs(rotationSpeedRaw) < MIN_SPEED) {
            rotationSpeedRaw = Math.copySign(MIN_SPEED, rotationSpeedRaw);
        }
        if (Math.abs(forwardSpeedRaw) > 0 && Math.abs(forwardSpeedRaw) < MIN_SPEED) {
            forwardSpeedRaw = Math.copySign(MIN_SPEED, forwardSpeedRaw);
        }
        if (Math.abs(strafeSpeedRaw) > 0 && Math.abs(strafeSpeedRaw) < MIN_SPEED) {
            strafeSpeedRaw = Math.copySign(MIN_SPEED, strafeSpeedRaw);
        }
        
        // Limit speeds
        double rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeedRaw));
        double forwardSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, forwardSpeedRaw));
        double strafeSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, strafeSpeedRaw));
        
        // Check if aligned using robot-center yaw error
        boolean lateralAligned = Math.abs(lateralOffset) < 0.01 || Math.abs(lateralError) < LATERAL_TOLERANCE;
        isAligned = Math.abs(robotCenterYawError) < YAW_TOLERANCE && Math.abs(distanceError) < DISTANCE_TOLERANCE && lateralAligned;
        
        // Apply drive commands
        if (isAligned) {
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        } else {
            drivetrain.setControl(driveRequest
                .withVelocityX(forwardSpeed)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(rotationSpeed));
        }
        
        // Update SmartDashboard
        SmartDashboard.putString("Auto Align Status", isAligned ? "ALIGNED" : "Aligning...");
        SmartDashboard.putString("Auto Align Camera", cameraUsed);
        SmartDashboard.putNumber("Auto Align Camera Yaw", targetYaw);
        SmartDashboard.putNumber("Auto Align Angle Offset", angleOffsetDeg);
        SmartDashboard.putNumber("Auto Align Robot Yaw Error", robotCenterYawError);
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
        // Command continues until button is released
        return false;
    }
}
