package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraFL;
    private final PhotonCamera cameraFR;
    
    // FL Camera data
    private double targetYawFL = 0.0;
    private double targetPitchFL = 0.0;
    private double targetAreaFL = 0.0;
    private double targetDistanceFL = 0.0;
    private int detectedTagIdFL = -1;
    private boolean targetVisibleFL = false;
    private int totalTargetsFL = 0;
    
    // FR Camera data
    private double targetYawFR = 0.0;
    private double targetPitchFR = 0.0;
    private double targetAreaFR = 0.0;
    private double targetDistanceFR = 0.0;
    private int detectedTagIdFR = -1;
    private boolean targetVisibleFR = false;
    private int totalTargetsFR = 0;

    private static final int DESIRED_TAG_ID = 14; // Primary AprilTag ID to track

    public VisionSubsystem() {
        // Initialize both cameras with names from Constants
        cameraFL = new PhotonCamera(Constants.Vision.kCameraNameFL);
        cameraFR = new PhotonCamera(Constants.Vision.kCameraNameFR);
    }

    @Override
    public void periodic() {
        // Process results for each camera independently
        processCameraFLResults();
        processCameraFRResults();
    }

    private void processCameraFLResults() {
        var results = cameraFL.getAllUnreadResults();

        // Reset values
        targetYawFL = 0.0;
        targetPitchFL = 0.0;
        targetAreaFL = 0.0;
        targetDistanceFL = 0.0;
        detectedTagIdFL = -1;
        targetVisibleFL = false;
        totalTargetsFL = 0;

        for (var result : results) {
            if (result.hasTargets()) {
                List<PhotonTrackedTarget> targets = result.getTargets();
                totalTargetsFL = targets.size();
                
                // First, try to find the desired tag
                PhotonTrackedTarget bestTarget = null;
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == DESIRED_TAG_ID) {
                        bestTarget = target;
                        break;
                    }
                }
                
                // If desired tag not found, use the best target (usually closest/largest)
                if (bestTarget == null && !targets.isEmpty()) {
                    bestTarget = targets.get(0);
                }
                
                // Extract data from best target
                if (bestTarget != null) {
                    targetVisibleFL = true;
                    detectedTagIdFL = bestTarget.getFiducialId();
                    targetYawFL = bestTarget.getYaw();
                    targetPitchFL = bestTarget.getPitch();
                    targetAreaFL = bestTarget.getArea();
                    
                    // Calculate approximate distance using target area (rough estimation)
                    // This is a simplified calculation - adjust based on your camera/tag setup
                    if (targetAreaFL > 0) {
                        targetDistanceFL = Math.sqrt(1.0 / targetAreaFL) * 10.0; // Rough approximation
                    }
                    
                    // If target has pose ambiguity, you can access it
                    // double ambiguity = bestTarget.getPoseAmbiguity();
                }
                break; // Use most recent result
            }
        }

        // Update SmartDashboard with comprehensive metrics for FL camera
        SmartDashboard.putBoolean("FL Target Visible", targetVisibleFL);
        SmartDashboard.putNumber("FL Detected Tag ID", detectedTagIdFL);
        SmartDashboard.putNumber("FL Target Yaw (deg)", targetYawFL);
        SmartDashboard.putNumber("FL Target Pitch (deg)", targetPitchFL);
        SmartDashboard.putNumber("FL Target Area (%)", targetAreaFL);
        SmartDashboard.putNumber("FL Approx Distance", targetDistanceFL);
        SmartDashboard.putNumber("FL Total Targets", totalTargetsFL);
        SmartDashboard.putBoolean("FL Camera Connected", cameraFL.isConnected());
        
        // Legacy outputs for compatibility
        SmartDashboard.putNumber("AprilTag " + DESIRED_TAG_ID + " Yaw FL", targetYawFL);
        SmartDashboard.putBoolean("Vision Target Visible FL", targetVisibleFL);
    }

    private void processCameraFRResults() {
        var results = cameraFR.getAllUnreadResults();

        // Reset values
        targetYawFR = 0.0;
        targetPitchFR = 0.0;
        targetAreaFR = 0.0;
        targetDistanceFR = 0.0;
        detectedTagIdFR = -1;
        targetVisibleFR = false;
        totalTargetsFR = 0;

        for (var result : results) {
            if (result.hasTargets()) {
                List<PhotonTrackedTarget> targets = result.getTargets();
                totalTargetsFR = targets.size();
                
                // First, try to find the desired tag
                PhotonTrackedTarget bestTarget = null;
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == DESIRED_TAG_ID) {
                        bestTarget = target;
                        break;
                    }
                }
                
                // If desired tag not found, use the best target (usually closest/largest)
                if (bestTarget == null && !targets.isEmpty()) {
                    bestTarget = targets.get(0);
                }
                
                // Extract data from best target
                if (bestTarget != null) {
                    targetVisibleFR = true;
                    detectedTagIdFR = bestTarget.getFiducialId();
                    targetYawFR = bestTarget.getYaw();
                    targetPitchFR = bestTarget.getPitch();
                    targetAreaFR = bestTarget.getArea();
                    
                    // Calculate approximate distance using target area (rough estimation)
                    if (targetAreaFR > 0) {
                        targetDistanceFR = Math.sqrt(1.0 / targetAreaFR) * 10.0; // Rough approximation
                    }
                }
                break; // Use most recent result
            }
        }

        // Update SmartDashboard with comprehensive metrics for FR camera
        SmartDashboard.putBoolean("FR Target Visible", targetVisibleFR);
        SmartDashboard.putNumber("FR Detected Tag ID", detectedTagIdFR);
        SmartDashboard.putNumber("FR Target Yaw (deg)", targetYawFR);
        SmartDashboard.putNumber("FR Target Pitch (deg)", targetPitchFR);
        SmartDashboard.putNumber("FR Target Area (%)", targetAreaFR);
        SmartDashboard.putNumber("FR Approx Distance", targetDistanceFR);
        SmartDashboard.putNumber("FR Total Targets", totalTargetsFR);
        SmartDashboard.putBoolean("FR Camera Connected", cameraFR.isConnected());
        
        // Legacy outputs for compatibility
        SmartDashboard.putNumber("AprilTag " + DESIRED_TAG_ID + " Yaw FR", targetYawFR);
        SmartDashboard.putBoolean("Vision Target Visible FR", targetVisibleFR);
    }

    // Getter methods for FL camera
    public double getTargetYawFL() {
        return targetYawFL;
    }

    public boolean isTargetVisibleFL() {
        return targetVisibleFL;
    }
    
    public int getDetectedTagIdFL() {
        return detectedTagIdFL;
    }
    
    public double getTargetDistanceFL() {
        return targetDistanceFL;
    }

    // Getter methods for FR camera
    public double getTargetYawFR() {
        return targetYawFR;
    }

    public boolean isTargetVisibleFR() {
        return targetVisibleFR;
    }
    
    public int getDetectedTagIdFR() {
        return detectedTagIdFR;
    }
    
    public double getTargetDistanceFR() {
        return targetDistanceFR;
    }
}
