package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    private int selectedTagId = 14; // Default tag ID

    // SmartDashboard chooser for tag selection
    private final SendableChooser<Integer> tagIdChooser;

    public VisionSubsystem() {
        // Initialize both cameras with names from Constants
        cameraFL = new PhotonCamera(Constants.Vision.kCameraNameFL);
        cameraFR = new PhotonCamera(Constants.Vision.kCameraNameFR);

        // Set up tag ID chooser
        tagIdChooser = new SendableChooser<>();
        tagIdChooser.setDefaultOption("Tag 14", 14);
        tagIdChooser.addOption("Tag 1", 1);
        tagIdChooser.addOption("Tag 2", 2);
        tagIdChooser.addOption("Tag 3", 3);
        tagIdChooser.addOption("Tag 4", 4);
        tagIdChooser.addOption("Tag 5", 5);
        tagIdChooser.addOption("Tag 6", 6);
        tagIdChooser.addOption("Tag 7", 7);
        tagIdChooser.addOption("Tag 8", 8);
        tagIdChooser.addOption("Tag 9", 9);
        tagIdChooser.addOption("Tag 10", 10);
        tagIdChooser.addOption("Tag 11", 11);
        tagIdChooser.addOption("Tag 12", 12);
        tagIdChooser.addOption("Tag 13", 13);
        tagIdChooser.addOption("Tag 15", 15);
        tagIdChooser.addOption("Tag 16", 16);
        SmartDashboard.putData("Vision Tag Selector", tagIdChooser);
    }

    @Override
    public void periodic() {
        // Update selected tag ID from chooser
        selectedTagId = tagIdChooser.getSelected();

        // Process results for each camera independently
        processCameraFLResults();
        processCameraFRResults();
        
        // Publish combined vision status for easy viewing
        publishCombinedStatus();
    }
    
    /**
     * Publishes combined vision status for easy dashboard viewing
     */
    private void publishCombinedStatus() {
        // Show which tag we're looking for
        SmartDashboard.putNumber("Target Tag ID", selectedTagId);
        
        // Show detected tags from both cameras
        String detectedTags = "";
        if (targetVisibleFL && targetVisibleFR) {
            detectedTags = "FL: Tag " + detectedTagIdFL + " | FR: Tag " + detectedTagIdFR;
        } else if (targetVisibleFL) {
            detectedTags = "FL: Tag " + detectedTagIdFL;
        } else if (targetVisibleFR) {
            detectedTags = "FR: Tag " + detectedTagIdFR;
        } else {
            detectedTags = "No Tags Detected";
        }
        SmartDashboard.putString("Detected Tags", detectedTags);
        
        // Show if target tag is visible
        boolean targetFound = (targetVisibleFL && detectedTagIdFL == selectedTagId) || 
                             (targetVisibleFR && detectedTagIdFR == selectedTagId);
        SmartDashboard.putBoolean("Target Tag Found", targetFound);
        
        // Show which camera sees the target
        String cameraSeeing = "";
        if (targetVisibleFL && detectedTagIdFL == selectedTagId && 
            targetVisibleFR && detectedTagIdFR == selectedTagId) {
            cameraSeeing = "Both Cameras";
        } else if (targetVisibleFL && detectedTagIdFL == selectedTagId) {
            cameraSeeing = "Front-Left Camera";
        } else if (targetVisibleFR && detectedTagIdFR == selectedTagId) {
            cameraSeeing = "Front-Right Camera";
        } else {
            cameraSeeing = "None";
        }
        SmartDashboard.putString("Target Visible On", cameraSeeing);
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
                
                // ONLY find the selected tag - do not use fallback
                PhotonTrackedTarget selectedTarget = null;
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == selectedTagId) {
                        selectedTarget = target;
                        break;
                    }
                }
                
                // Extract data ONLY if we found the selected tag
                if (selectedTarget != null) {
                    targetVisibleFL = true;
                    detectedTagIdFL = selectedTarget.getFiducialId();
                    targetYawFL = selectedTarget.getYaw();
                    targetPitchFL = selectedTarget.getPitch();
                    targetAreaFL = selectedTarget.getArea();
                    
                    // Calculate approximate distance using target area (rough estimation)
                    // This is a simplified calculation - adjust based on your camera/tag setup
                    if (targetAreaFL > 0) {
                        targetDistanceFL = Math.sqrt(1.0 / targetAreaFL) * 10.0; // Rough approximation
                    }
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
        SmartDashboard.putNumber("AprilTag " + selectedTagId + " Yaw FL", targetYawFL);
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
                
                // ONLY find the selected tag - do not use fallback
                PhotonTrackedTarget selectedTarget = null;
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == selectedTagId) {
                        selectedTarget = target;
                        break;
                    }
                }
                
                // Extract data ONLY if we found the selected tag
                if (selectedTarget != null) {
                    targetVisibleFR = true;
                    detectedTagIdFR = selectedTarget.getFiducialId();
                    targetYawFR = selectedTarget.getYaw();
                    targetPitchFR = selectedTarget.getPitch();
                    targetAreaFR = selectedTarget.getArea();
                    
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
        SmartDashboard.putNumber("AprilTag " + selectedTagId + " Yaw FR", targetYawFR);
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

    // Public getter for selected tag ID
    public int getSelectedTagId() {
        return selectedTagId;
    }
}
