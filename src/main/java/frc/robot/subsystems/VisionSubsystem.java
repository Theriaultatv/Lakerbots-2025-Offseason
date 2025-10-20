package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraFL;
    private final PhotonCamera cameraFR;
    private double targetYawFL = 0.0;
    private boolean targetVisibleFL = false;
    private double targetYawFR = 0.0;
    private boolean targetVisibleFR = false;

    private static final int DESIRED_TAG_ID = 14; // Specify the AprilTag ID here

    public VisionSubsystem() {
        // Initialize both cameras with correct names
        cameraFL = new PhotonCamera("photonvisionFL");
        cameraFR = new PhotonCamera("photonvisionFR");
    }

    @Override
    public void periodic() {
        // Process results for each camera independently
        processCameraFLResults();
        processCameraFRResults();
    }

    private void processCameraFLResults() {
        var results = cameraFL.getAllUnreadResults();

        targetYawFL = 0.0;
        targetVisibleFL = false;

        for (var result : results) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == DESIRED_TAG_ID) {
                        targetVisibleFL = true;
                        targetYawFL = target.getYaw();
                        break;
                    }
                }
            }
            if (targetVisibleFL) {
                break;
            }
        }

        // Update the SmartDashboard with the results for the front-left camera
        SmartDashboard.putNumber("AprilTag " + DESIRED_TAG_ID + " Yaw FL", targetYawFL);
        SmartDashboard.putBoolean("Vision Target Visible FL", targetVisibleFL);
    }

    private void processCameraFRResults() {
        var results = cameraFR.getAllUnreadResults();

        targetYawFR = 0.0;
        targetVisibleFR = false;

        for (var result : results) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == DESIRED_TAG_ID) {
                        targetVisibleFR = true;
                        targetYawFR = target.getYaw();
                        break;
                    }
                }
            }
            if (targetVisibleFR) {
                break;
            }
        }

        // Update the SmartDashboard with the results for the front-right camera
        SmartDashboard.putNumber("AprilTag " + DESIRED_TAG_ID + " Yaw FR", targetYawFR);
        SmartDashboard.putBoolean("Vision Target Visible FR", targetVisibleFR);
    }

    public double getTargetYawFL() {
        return targetYawFL;
    }

    public boolean isTargetVisibleFL() {
        return targetVisibleFL;
    }

    public double getTargetYawFR() {
        return targetYawFR;
    }

    public boolean isTargetVisibleFR() {
        return targetVisibleFR;
    }
}