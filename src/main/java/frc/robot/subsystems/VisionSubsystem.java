package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraFL;
    private final PhotonCamera cameraFR;
    
    // Pose estimators for each camera
    private final PhotonPoseEstimator poseEstimatorFL;
    private final PhotonPoseEstimator poseEstimatorFR;
    
    // Latest estimated poses
    private Optional<EstimatedRobotPose> latestEstimatedPoseFL = Optional.empty();
    private Optional<EstimatedRobotPose> latestEstimatedPoseFR = Optional.empty();

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
        
        // Initialize pose estimators with MULTI_TAG_PNP_ON_COPROCESSOR strategy
        // This uses multiple tags when available for better accuracy
        poseEstimatorFL = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.Vision.kRobotToCamFL
        );
        poseEstimatorFL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        poseEstimatorFR = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.Vision.kRobotToCamFR
        );
        poseEstimatorFR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
        tagIdChooser.addOption("Tag 17", 17);
        tagIdChooser.addOption("Tag 18", 18);
        tagIdChooser.addOption("Tag 19", 19);
        tagIdChooser.addOption("Tag 20", 20);
        tagIdChooser.addOption("Tag 21", 21);
        tagIdChooser.addOption("Tag 22", 22);
        tagIdChooser.addOption("Tag 23", 23);
        tagIdChooser.addOption("Tag 24", 24);
        SmartDashboard.putData("Vision Tag Selector", tagIdChooser);
    }

    @Override
    public void periodic() {
        // Update selected tag ID from chooser
        selectedTagId = tagIdChooser.getSelected();

        // Update pose estimates for both cameras
        updatePoseEstimates();

        // Process results for each camera independently
        processCameraFLResults();
        processCameraFRResults();
        
        // Publish combined vision status for easy viewing
        publishCombinedStatus();
        
        // Publish pose estimation data to dashboard
        publishPoseEstimationData();
    }
    
    /**
     * Updates pose estimates from both cameras
     */
    private void updatePoseEstimates() {
        // Update FL camera pose estimate
        var resultFL = cameraFL.getLatestResult();
        if (resultFL.hasTargets()) {
            latestEstimatedPoseFL = poseEstimatorFL.update(resultFL);
        } else {
            latestEstimatedPoseFL = Optional.empty();
        }
        
        // Update FR camera pose estimate
        var resultFR = cameraFR.getLatestResult();
        if (resultFR.hasTargets()) {
            latestEstimatedPoseFR = poseEstimatorFR.update(resultFR);
        } else {
            latestEstimatedPoseFR = Optional.empty();
        }
    }
    
    /**
     * Publishes pose estimation data to SmartDashboard for Elastic dashboard
     */
    private void publishPoseEstimationData() {
        // FL Camera pose estimation
        if (latestEstimatedPoseFL.isPresent()) {
            EstimatedRobotPose estimatedPose = latestEstimatedPoseFL.get();
            Pose2d pose = estimatedPose.estimatedPose.toPose2d();
            
            SmartDashboard.putBoolean("Vision/FL/Pose Valid", true);
            SmartDashboard.putNumber("Vision/FL/Pose X", pose.getX());
            SmartDashboard.putNumber("Vision/FL/Pose Y", pose.getY());
            SmartDashboard.putNumber("Vision/FL/Pose Rotation", pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/FL/Pose Timestamp", estimatedPose.timestampSeconds);
            SmartDashboard.putNumber("Vision/FL/Tags Used", estimatedPose.targetsUsed.size());
            
            // Publish tag IDs used
            StringBuilder tagIds = new StringBuilder();
            for (var target : estimatedPose.targetsUsed) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(target.getFiducialId());
            }
            SmartDashboard.putString("Vision/FL/Tags Used IDs", tagIds.toString());
        } else {
            SmartDashboard.putBoolean("Vision/FL/Pose Valid", false);
        }
        
        // FR Camera pose estimation
        if (latestEstimatedPoseFR.isPresent()) {
            EstimatedRobotPose estimatedPose = latestEstimatedPoseFR.get();
            Pose2d pose = estimatedPose.estimatedPose.toPose2d();
            
            SmartDashboard.putBoolean("Vision/FR/Pose Valid", true);
            SmartDashboard.putNumber("Vision/FR/Pose X", pose.getX());
            SmartDashboard.putNumber("Vision/FR/Pose Y", pose.getY());
            SmartDashboard.putNumber("Vision/FR/Pose Rotation", pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/FR/Pose Timestamp", estimatedPose.timestampSeconds);
            SmartDashboard.putNumber("Vision/FR/Tags Used", estimatedPose.targetsUsed.size());
            
            // Publish tag IDs used
            StringBuilder tagIds = new StringBuilder();
            for (var target : estimatedPose.targetsUsed) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(target.getFiducialId());
            }
            SmartDashboard.putString("Vision/FR/Tags Used IDs", tagIds.toString());
        } else {
            SmartDashboard.putBoolean("Vision/FR/Pose Valid", false);
        }
        
        // Combined status
        boolean anyPoseValid = latestEstimatedPoseFL.isPresent() || latestEstimatedPoseFR.isPresent();
        SmartDashboard.putBoolean("Vision/Any Pose Valid", anyPoseValid);
        
        int totalTagsUsed = 0;
        if (latestEstimatedPoseFL.isPresent()) {
            totalTagsUsed += latestEstimatedPoseFL.get().targetsUsed.size();
        }
        if (latestEstimatedPoseFR.isPresent()) {
            totalTagsUsed += latestEstimatedPoseFR.get().targetsUsed.size();
        }
        SmartDashboard.putNumber("Vision/Total Tags Used", totalTagsUsed);
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
    
    /**
     * Gets the latest estimated robot pose from the FL camera
     * @return Optional containing the estimated pose, or empty if no estimate available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFL() {
        return latestEstimatedPoseFL;
    }
    
    /**
     * Gets the latest estimated robot pose from the FR camera
     * @return Optional containing the estimated pose, or empty if no estimate available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFR() {
        return latestEstimatedPoseFR;
    }
    
    /**
     * Sets the reference pose for the pose estimators.
     * This should be called when resetting odometry.
     * @param pose The new reference pose
     */
    public void setReferencePose(Pose2d pose) {
        poseEstimatorFL.setReferencePose(pose);
        poseEstimatorFR.setReferencePose(pose);
    }
}
