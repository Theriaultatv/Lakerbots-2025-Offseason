package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private double targetYaw = 0.0;
    private boolean targetVisible = false;

    private static final int DESIRED_TAG_ID = 14; // Specify the AprilTag ID here

    public VisionSubsystem() {
        // Replace "photonvision" with your camera's name on the web interface
        camera = new PhotonCamera("photonvision");
    }

    @Override
    public void periodic() {
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();

        // Reset the values each periodic loop
        targetVisible = false;
        targetYaw = 0.0;

        // Check if the result has any targets
        if (result.hasTargets()) {
            // Loop through all detected targets
            for (PhotonTrackedTarget target : result.getTargets()) {
                // Find the target with the specific ID
                if (target.getFiducialId() == DESIRED_TAG_ID) {
                    targetVisible = true;
                    targetYaw = target.getYaw();
                    break; // Exit the loop once the target is found
                }
            }
        }

        // Send the yaw and visibility status for the specific target to the Smart Dashboard
        SmartDashboard.putNumber("AprilTag " + DESIRED_TAG_ID + " Yaw", targetYaw);
        SmartDashboard.putBoolean("AprilTag " + DESIRED_TAG_ID + " Visible", targetVisible);
    }

    public double getTargetYaw() {
        return targetYaw;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }
}