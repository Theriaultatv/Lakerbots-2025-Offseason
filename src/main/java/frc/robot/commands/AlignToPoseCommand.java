package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Command to drive the robot to a specific pose (X, Y, Heading) on the field.
 * Uses PID controllers for X, Y, and rotation to smoothly navigate to the target.
 */
public class AlignToPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    
    // Target pose - can be set from dashboard
    private double targetX;
    private double targetY;
    private double targetHeadingDegrees;
    
    // PID Controllers for X, Y, and rotation
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // PID Constants (tuned for smooth movement)
    private static final double XY_KP = 2.0;
    private static final double XY_KI = 0.0;
    private static final double XY_KD = 0.1;
    
    private static final double ROTATION_KP = 0.05;
    private static final double ROTATION_KI = 0.0;
    private static final double ROTATION_KD = 0.005;
    
    // Tolerance values
    private static final double POSITION_TOLERANCE = 0.02; // meters
    private static final double ROTATION_TOLERANCE = 0.5; // degrees
    
    // Speed limits
    private static final double MAX_SPEED = 2.0; // m/s
    private static final double MAX_ROTATION_SPEED = 3.0; // rad/s
    
    /**
     * Creates an AlignToPoseCommand with a specific target pose.
     * 
     * @param drivetrain The swerve drivetrain
     * @param targetX Target X position in meters
     * @param targetY Target Y position in meters
     * @param targetHeadingDegrees Target heading in degrees
     */
    public AlignToPoseCommand(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, double targetHeadingDegrees) {
        this.drivetrain = drivetrain;
        this.driveRequest = new SwerveRequest.FieldCentric();
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeadingDegrees = targetHeadingDegrees;
        
        // Initialize PID controllers
        xController = new PIDController(XY_KP, XY_KI, XY_KD);
        yController = new PIDController(XY_KP, XY_KI, XY_KD);
        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        
        // Configure rotation controller for continuous input (-180 to 180 degrees)
        rotationController.enableContinuousInput(-180, 180);
        
        // Set tolerances
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotationController.setTolerance(ROTATION_TOLERANCE);
        
        addRequirements(drivetrain);
    }
    
    /**
     * Creates an AlignToPoseCommand that reads target from SmartDashboard.
     * Default target: (6.66, 4.36, -177.5)
     * 
     * @param drivetrain The swerve drivetrain
     */
    public AlignToPoseCommand(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, 6.66, 4.36, -177.5);
    }

    @Override
    public void initialize() {
        // Read target from SmartDashboard if available
        targetX = SmartDashboard.getNumber("Target Pose/X", targetX);
        targetY = SmartDashboard.getNumber("Target Pose/Y", targetY);
        targetHeadingDegrees = SmartDashboard.getNumber("Target Pose/Heading", targetHeadingDegrees);
        
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        // Set setpoints
        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        rotationController.setSetpoint(targetHeadingDegrees);
        
        // Publish status
        SmartDashboard.putBoolean("Align To Pose/Active", true);
        SmartDashboard.putString("Align To Pose/Status", "Driving to pose...");
        
        System.out.println(String.format("AlignToPoseCommand: Driving to (%.2f, %.2f, %.1f°)", 
            targetX, targetY, targetHeadingDegrees));
    }

    @Override
    public void execute() {
        // Get current pose
        Pose2d currentPose = drivetrain.getState().Pose;
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeadingDegrees = currentPose.getRotation().getDegrees();
        
        // Calculate PID outputs
        double xSpeed = xController.calculate(currentX);
        double ySpeed = yController.calculate(currentY);
        double rotationSpeed = rotationController.calculate(currentHeadingDegrees);
        
        // Limit speeds
        xSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, xSpeed));
        ySpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, ySpeed));
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeed));
        
        // Apply drive commands (field-centric)
        drivetrain.setControl(driveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rotationSpeed));
        
        // Calculate errors for dashboard
        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double headingError = targetHeadingDegrees - currentHeadingDegrees;
        
        // Normalize heading error to -180 to 180
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        double distanceError = Math.sqrt(xError * xError + yError * yError);
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Align To Pose/Current X", currentX);
        SmartDashboard.putNumber("Align To Pose/Current Y", currentY);
        SmartDashboard.putNumber("Align To Pose/Current Heading", currentHeadingDegrees);
        SmartDashboard.putNumber("Align To Pose/X Error", xError);
        SmartDashboard.putNumber("Align To Pose/Y Error", yError);
        SmartDashboard.putNumber("Align To Pose/Heading Error", headingError);
        SmartDashboard.putNumber("Align To Pose/Distance Error", distanceError);
        SmartDashboard.putNumber("Align To Pose/X Speed", xSpeed);
        SmartDashboard.putNumber("Align To Pose/Y Speed", ySpeed);
        SmartDashboard.putNumber("Align To Pose/Rotation Speed", rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        
        SmartDashboard.putBoolean("Align To Pose/Active", false);
        SmartDashboard.putString("Align To Pose/Status", interrupted ? "Interrupted" : "Complete");
        
        if (!interrupted) {
            System.out.println("AlignToPoseCommand: Successfully reached target pose!");
        } else {
            System.out.println("AlignToPoseCommand: Command interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes when all controllers are at setpoint
        boolean atTarget = xController.atSetpoint() && 
                          yController.atSetpoint() && 
                          rotationController.atSetpoint();
        
        if (atTarget) {
            SmartDashboard.putString("Align To Pose/Status", "At target!");
        }
        
        return atTarget;
    }
}
