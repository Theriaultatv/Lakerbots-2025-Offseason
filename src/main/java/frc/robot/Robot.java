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

import frc.robot.Constants.Swerve;

public class Robot extends TimedRobot {
    private XboxController controller;
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        // Controller (if needed separately from RobotContainer)
        controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

        // Create a network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient3("10.80.46.11"); // IP of the Orange Pi
        CameraServer.addServer("http://10.80.46.11:1181/stream.mjpg"); // FL Camera
        CameraServer.addServer("http://10.80.46.11:1184/stream.mjpg"); // FR Camera
        NetworkTable visionTable = inst.getTable("photonvision");
        NetworkTableEntry targetYaw2 = visionTable.getEntry("targetYaw2");
            // targetYaw2 = target.getYaw();
            // Now you can grab the table published by the Orange PI
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        
        // Update vision measurements for pose estimation
        m_robotContainer.updateVisionMeasurements();
        
        // Update Field2D with current robot pose and vision targets
        m_robotContainer.updateField2d();
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
        // Vision data is now handled by VisionSubsystem and automatically updated to SmartDashboard
        // Get vision data from VisionSubsystem through RobotContainer if needed
        var visionSubsystem = m_robotContainer.getVisionSubsystem();
        
        // Get data from both cameras independently
        boolean targetVisibleFL = visionSubsystem.isTargetVisibleFL();
        double targetYawFL = visionSubsystem.getTargetYawFL();
        boolean targetVisibleFR = visionSubsystem.isTargetVisibleFR();
        double targetYawFR = visionSubsystem.getTargetYawFR();

        // Output all Xbox controller button states to SmartDashboard
        SmartDashboard.putBoolean("Controller A Button", controller.getAButton());
        SmartDashboard.putBoolean("Controller B Button", controller.getBButton());
        SmartDashboard.putBoolean("Controller X Button", controller.getXButton());
        SmartDashboard.putBoolean("Controller Y Button", controller.getYButton());
        SmartDashboard.putBoolean("Controller Left Bumper", controller.getLeftBumperButton());
        SmartDashboard.putBoolean("Controller Right Bumper", controller.getRightBumperButton());
        SmartDashboard.putBoolean("Controller Back Button", controller.getBackButton());
        SmartDashboard.putBoolean("Controller Start Button", controller.getStartButton());
        SmartDashboard.putBoolean("Controller Left Stick Button", controller.getLeftStickButton());
        SmartDashboard.putBoolean("Controller Right Stick Button", controller.getRightStickButton());
        
        // Output all Xbox controller axis values to SmartDashboard
        SmartDashboard.putNumber("Controller Left Stick X", controller.getLeftX());
        SmartDashboard.putNumber("Controller Left Stick Y", controller.getLeftY());
        SmartDashboard.putNumber("Controller Right Stick X", controller.getRightX());
        SmartDashboard.putNumber("Controller Right Stick Y", controller.getRightY());
        SmartDashboard.putNumber("Controller Left Trigger", controller.getLeftTriggerAxis());
        SmartDashboard.putNumber("Controller Right Trigger", controller.getRightTriggerAxis());
        
        // Output D-Pad (POV) state
        SmartDashboard.putNumber("Controller POV", controller.getPOV());

        // OPTIONAL: Auto-turn using FL camera when 'A' button is held
        if (controller.getAButton() && targetVisibleFL) {
            double turnCommand = -1.0 * targetYawFL * 0.02 * Swerve.kMaxAngularSpeed;
            SmartDashboard.putNumber("Auto Turn Command FL", turnCommand);
            // If you want to inject this into drivetrain, you'd need access
            // to drivetrain object, or push this to NetworkTables / a global state
        }
        
        // OPTIONAL: Auto-turn using FR camera when 'B' button is held
        if (controller.getBButton() && targetVisibleFR) {
            double turnCommand = -1.0 * targetYawFR * 0.02 * Swerve.kMaxAngularSpeed;
            SmartDashboard.putNumber("Auto Turn Command FR", turnCommand);
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
