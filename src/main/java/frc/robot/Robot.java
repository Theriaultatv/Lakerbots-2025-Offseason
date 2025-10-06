package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.PhotonCamera;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Vision;

public class Robot extends TimedRobot {
    private PhotonCamera cameraFL;
    private PhotonCamera cameraFR;
    private XboxController controller;
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        // Controller (if needed separately from RobotContainer)
        controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

        // Set up PhotonVision camera
        cameraFL = new PhotonCamera(Vision.kCameraNameFL);
        cameraFR = new PhotonCamera(Vision.kCameraNameFR);

        // Create a network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient3("10.80.46.11"); // IP of the Orange Pi
        CameraServer.addServer("http://10.80.46.11:1181/stream.mjpg"); // FL Camera
        CameraServer.addServer("http://10.80.46.11:1184/stream.mjpg"); // FR Camera
        NetworkTable visionTable = inst.getTable("photonvision");
        NetworkTableEntry targetYawFR = visionTable.getEntry("targetYaw2");
            // targetYaw2 = target.getYaw();
            // Now you can grabe the table published by the Orange PI
            

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        boolean targetVisibleFL = false;
        double targetYawFL = 0.0;

        var resultFL = cameraFL.getLatestResult();
        if (resultFL.hasTargets()) {
            for (var targetFL : resultFL.getTargets()) {
                if (targetFL.getFiducialId() == 14) {
                    targetYawFL = targetFL.getYaw();
                    targetVisibleFL = true;
                    //targetResult = result.getTargets;
                    break;
                }
            }
        }
        boolean targetVisibleFR = false;
        double targetYawFR = 0.0;

        var resultFR = cameraFR.getLatestResult();
        if (resultFR.hasTargets()) {
            for (var targetFR : resultFR.getTargets()) {
                if (targetFR.getFiducialId() == 14) {
                    targetYawFR = targetFR.getYaw();
                    targetVisibleFR = true;
                    //targetResult = result.getTargets;
                    break;
                }
            }
        }
        // SmartDashboard output
        SmartDashboard.putBoolean("Vision Target Visible FL", targetVisibleFL);
        SmartDashboard.putBoolean("Vision Target Visible FR", targetVisibleFR);
        //SmartDashboard.putNumber("Targets Result", targetResult);
        SmartDashboard.putNumber("Target Yaw FL", targetYawFL);
        SmartDashboard.putNumber("Target Yaw FR", targetYawFR);
        
        

        // Camera Feeds
        //SmartDashboard.putData("FL Camera", "http://10.80.46.11:1181/stream.mjpg");

        // OPTIONAL: If you want to override the driver's turn when 'A' is held
        if (controller.getAButton() && targetVisibleFL) {
            double turnCommand = -1.0 * targetYawFL * 0.02 * Swerve.kMaxAngularSpeed;
            SmartDashboard.putNumber("Auto Turn Command", turnCommand);
            // If you want to inject this into drivetrain, you'd need access
            // to drivetrain object, or push this to NetworkTables / a global state
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
