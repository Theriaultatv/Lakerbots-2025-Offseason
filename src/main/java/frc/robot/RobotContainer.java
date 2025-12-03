// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Filesystem;

//import java.io.File;

//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PumpkinSubsystem;
import frc.robot.commands.AlignToAprilTagCommand;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.RunPumpkinCommand;

import java.util.Optional;

public class RobotContainer {
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final PumpkinSubsystem pumpkinSubsystem = new PumpkinSubsystem();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Field2D for visualization on Elastic dashboard
    private final Field2d field2d = new Field2d();
    
    // Autonomous chooser for PathPlanner
    private final SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        // Configure PathPlanner AutoBuilder FIRST - this is required for PathPlannerAuto to work
        configurePathPlanner();
        
        // Register named commands for PathPlanner BEFORE building auto chooser
        registerNamedCommands();
        
        // Build auto chooser - this will create a chooser with all .auto files
        autoChooser = buildAutoChooser();
        
        // Publish auto chooser to SmartDashboard for Elastic dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Initialize Target Pose dashboard values
        initializeTargetPoseDashboard();
        
        configureBindings();
        
        // Publish Field2D to SmartDashboard for Elastic dashboard
        // Using "Field2d" as the key name for better Elastic compatibility
        SmartDashboard.putData("Field2d", field2d);
    }
    
    /**
     * Initializes the Target Pose dashboard values.
     * These can be modified on the Elastic dashboard to set custom target poses.
     */
    private void initializeTargetPoseDashboard() {
        // Set default target pose: (6.66, 4.36, -177.5)
        SmartDashboard.putNumber("Target Pose/X", 6.66);
        SmartDashboard.putNumber("Target Pose/Y", 4.36);
        SmartDashboard.putNumber("Target Pose/Heading", -177.5);
        
        System.out.println("Target Pose dashboard initialized:");
        System.out.println("  Default X: 6.66 meters");
        System.out.println("  Default Y: 4.36 meters");
        System.out.println("  Default Heading: -177.5 degrees");
        System.out.println("You can modify these values on the Elastic dashboard");
    }
    
    /**
     * Configures PathPlanner AutoBuilder for autonomous path following.
     * This MUST be called before creating any PathPlannerAuto commands.
     */
    private void configurePathPlanner() {
        try {
            System.out.println("========================================");
            System.out.println("Configuring PathPlanner AutoBuilder...");
            System.out.println("========================================");
            
            // Load robot configuration from PathPlanner config file
            RobotConfig config;
            try {
                config = RobotConfig.fromGUISettings();
                System.out.println("✓ Loaded PathPlanner robot config from GUI settings");
            } catch (Exception e) {
                System.err.println("✗ Failed to load PathPlanner config from GUI settings");
                System.err.println("  Error: " + e.getMessage());
                System.err.println("  You need to configure your robot in PathPlanner GUI");
                System.err.println("  PathPlannerAuto commands may not work correctly!");
                config = null;
            }
            
            // Configure AutoBuilder with the drivetrain
            // Using simplified configuration for CTRE Phoenix 6 swerve
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose, // Supplier of current robot pose
                (pose) -> {
                    // Reset pose using CTRE's resetPose method
                    // Note: CTRE Phoenix 6 swerve uses resetPose(Pose2d) from parent class
                    try {
                        drivetrain.resetPose(pose);
                    } catch (Exception e) {
                        System.err.println("Failed to reset pose: " + e.getMessage());
                    }
                }, // Consumer to reset odometry
                () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
                (speeds, feedforwards) -> drivetrain.setControl(
                    new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                ), // Consumer to apply robot speeds
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
                ),
                config, // Robot configuration (can be null if not using GUI settings)
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                drivetrain // The drive subsystem
            );
            
            System.out.println("✓ PathPlanner AutoBuilder configured successfully");
            System.out.println("========================================");
            
        } catch (Exception e) {
            System.err.println("========================================");
            System.err.println("✗ CRITICAL: Failed to configure PathPlanner!");
            System.err.println("  Error: " + e.getMessage());
            e.printStackTrace();
            System.err.println("  PathPlannerAuto commands will not work!");
            System.err.println("========================================");
        }
    }
    
    /**
     * Builds the autonomous chooser with PathPlanner autos.
     * Manually loads autos since AutoBuilder requires full configuration.
     */
    private SendableChooser<Command> buildAutoChooser() {
        System.out.println("========================================");
        System.out.println("Building Auto Chooser...");
        System.out.println("========================================");
        
        SendableChooser<Command> chooser = new SendableChooser<>();
        
        // Set default option
        chooser.setDefaultOption("Do Nothing", Commands.none());
        System.out.println("✓ Added default: Do Nothing");
        
        // Manually add PathPlanner autos
        // These will load from deploy/pathplanner/autos/
        
        // Add Vision Test auto
        try {
            Command visionTestAuto = new PathPlannerAuto("Vision Test");
            chooser.addOption("Vision Test", visionTestAuto);
            System.out.println("✓ Successfully loaded: Vision Test");
        } catch (Exception e) {
            System.err.println("✗ Failed to load Vision Test auto:");
            System.err.println("  Error: " + e.getMessage());
            e.printStackTrace();
        }
        
        // Add Example Auto
        try {
            Command exampleAuto = new PathPlannerAuto("Example Auto");
            chooser.addOption("Example Auto", exampleAuto);
            System.out.println("✓ Successfully loaded: Example Auto");
        } catch (Exception e) {
            System.err.println("✗ Failed to load Example Auto:");
            System.err.println("  Error: " + e.getMessage());
            e.printStackTrace();
        }
        
        // Add Vision Center Auto
        try {
            Command visionCenterAuto = new PathPlannerAuto("Vision Center");
            chooser.addOption("Vision Center", visionCenterAuto);
            System.out.println("✓ Successfully loaded: Vision Center");
        } catch (Exception e) {
            System.err.println("✗ Failed to load Vision Center auto:");
            System.err.println("  Error: " + e.getMessage());
            e.printStackTrace();
        }
        
        // Add more autos here as you create them in PathPlanner
        // Just copy the pattern above:
        // try {
        //     Command yourAuto = new PathPlannerAuto("Your Auto Name");
        //     chooser.addOption("Your Auto Name", yourAuto);
        //     System.out.println("✓ Successfully loaded: Your Auto Name");
        // } catch (Exception e) {
        //     System.err.println("✗ Failed to load Your Auto Name:");
        //     System.err.println("  Error: " + e.getMessage());
        //     e.printStackTrace();
        // }
        
        System.out.println("========================================");
        System.out.println("Auto Chooser Build Complete");
        System.out.println("========================================");
        
        return chooser;
    }
    
    /**
     * Registers named commands that can be used in PathPlanner autos.
     * Add your custom commands here to use them in PathPlanner GUI.
     * These commands can be triggered at specific points along paths.
     */
    private void registerNamedCommands() {
        // Example: Register a command to align to AprilTag
        NamedCommands.registerCommand("AlignToAprilTagCommand", 
            new AlignToAprilTagCommand(drivetrain, visionSubsystem, 0.0));

        NamedCommands.registerCommand("RunPumpkinCommand",
            new RunPumpkinCommand(pumpkinSubsystem));
        
        // Example: Register a command to print a message
        NamedCommands.registerCommand("PrintMessage", 
            Commands.print("PathPlanner command executed!"));
        
        // Add more named commands here as needed
        // These will be available in the PathPlanner GUI
        // NamedCommands.registerCommand("YourCommandName", yourCommand);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        // Reset the field-centric heading when start button is pressed
        joystick.start().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            // Also update vision pose estimators with the new pose
            Pose2d currentPose = drivetrain.getState().Pose;
            visionSubsystem.setReferencePose(currentPose);
            SmartDashboard.putString("Field-Centric Status", "Reset at: " + System.currentTimeMillis());
            System.out.println("Field-centric heading reset! Current pose: " + currentPose);
        }));

        // Auto-align to AprilTag command with pumpkin motor sequence:
        // Left bumper: Align to center of tag, then run pumpkin motor for 3 seconds
        joystick.leftBumper().onTrue(
            new AlignToAprilTagCommand(drivetrain, visionSubsystem, 0.0)
                .andThen(new RunPumpkinCommand(pumpkinSubsystem).withTimeout(Constants.Pumpkin.kRunDuration))
        );

        // Right bumper: Drive to target pose (reads from dashboard)
        joystick.rightBumper().onTrue(new AlignToPoseCommand(drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Periodic method to update Field2D with robot pose and vision targets.
     * This should be called from Robot.robotPeriodic() or similar.
     */
    public void updateField2d() {
        // Update robot pose on field
        Pose2d currentPose = drivetrain.getState().Pose;
        field2d.setRobotPose(currentPose);
        
        // Update AprilTag visualization based on detected tags
        updateAprilTagVisualization();
    }
    
    /**
     * Updates the drivetrain's pose estimator with vision measurements.
     * This should be called periodically (e.g., from Robot.robotPeriodic()).
     */
    public void updateVisionMeasurements() {
        // Process FL camera vision measurement
        var poseFL = visionSubsystem.getEstimatedGlobalPoseFL();
        if (poseFL.isPresent()) {
            var estimatedPose = poseFL.get();
            Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
            
            // Determine standard deviation based on number of tags
            var stdDevs = estimatedPose.targetsUsed.size() >= 2 
                ? Constants.Vision.kMultiTagStdDevs 
                : Constants.Vision.kSingleTagStdDevs;
            
            // Add vision measurement to drivetrain
            drivetrain.addVisionMeasurement(
                pose2d,
                estimatedPose.timestampSeconds,
                stdDevs
            );
            
            // Publish to dashboard for debugging
            SmartDashboard.putString("Vision/FL/Measurement Status", "Applied");
        } else {
            SmartDashboard.putString("Vision/FL/Measurement Status", "No Pose");
        }
        
        // Process FR camera vision measurement
        var poseFR = visionSubsystem.getEstimatedGlobalPoseFR();
        if (poseFR.isPresent()) {
            var estimatedPose = poseFR.get();
            Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
            
            // Determine standard deviation based on number of tags
            var stdDevs = estimatedPose.targetsUsed.size() >= 2 
                ? Constants.Vision.kMultiTagStdDevs 
                : Constants.Vision.kSingleTagStdDevs;
            
            // Add vision measurement to drivetrain
            drivetrain.addVisionMeasurement(
                pose2d,
                estimatedPose.timestampSeconds,
                stdDevs
            );
            
            // Publish to dashboard for debugging
            SmartDashboard.putString("Vision/FR/Measurement Status", "Applied");
        } else {
            SmartDashboard.putString("Vision/FR/Measurement Status", "No Pose");
        }
    }

    /**
     * Updates Field2D with detected AprilTag positions.
     * Shows which tags are currently visible to the cameras.
     */
    private void updateAprilTagVisualization() {
        // Get detected tag IDs from both cameras
        int detectedTagFL = visionSubsystem.getDetectedTagIdFL();
        int detectedTagFR = visionSubsystem.getDetectedTagIdFR();
        
        // Clear previous tag poses
        field2d.getObject("Detected Tags FL").setPoses();
        field2d.getObject("Detected Tags FR").setPoses();
        
        // Add FL camera detected tag
        if (detectedTagFL > 0 && visionSubsystem.isTargetVisibleFL()) {
            Optional<Pose2d> tagPose = getAprilTagPose(detectedTagFL);
            if (tagPose.isPresent()) {
                field2d.getObject("Detected Tags FL").setPose(tagPose.get());
            }
        }
        
        // Add FR camera detected tag
        if (detectedTagFR > 0 && visionSubsystem.isTargetVisibleFR()) {
            Optional<Pose2d> tagPose = getAprilTagPose(detectedTagFR);
            if (tagPose.isPresent()) {
                field2d.getObject("Detected Tags FR").setPose(tagPose.get());
            }
        }
    }

    /**
     * Gets the 2D pose of an AprilTag from the field layout.
     * 
     * @param tagId The AprilTag ID
     * @return Optional containing the tag's Pose2d if found
     */
    private Optional<Pose2d> getAprilTagPose(int tagId) {
        try {
            var tagPose3d = Constants.Vision.kTagLayout.getTagPose(tagId);
            if (tagPose3d.isPresent()) {
                // Convert Pose3d to Pose2d (just use X, Y, and rotation around Z)
                var pose3d = tagPose3d.get();
                return Optional.of(new Pose2d(
                    pose3d.getX(),
                    pose3d.getY(),
                    pose3d.getRotation().toRotation2d()
                ));
            }
        } catch (Exception e) {
            // Tag not found in layout
        }
        return Optional.empty();
    }

    /**
     * Gets the Field2d object for external access if needed.
     * 
     * @return The Field2d object
     */
    public Field2d getField2d() {
        return field2d;
    }

    public Command getAutonomousCommand() {
        System.out.println("========================================");
        System.out.println("Getting Autonomous Command...");
        
        // Return the selected autonomous command from the chooser
        Command selectedAuto = autoChooser.getSelected();
        
        if (selectedAuto != null) {
            System.out.println("✓ Selected auto command retrieved successfully");
            System.out.println("========================================");
            return selectedAuto;
        }
        
        // Fallback if no auto is selected
        System.err.println("✗ No autonomous command selected!");
        System.out.println("========================================");
        return Commands.print("No autonomous command selected");
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public PumpkinSubsystem getPumpkinSubsystem() {
        return pumpkinSubsystem;
    }
}
  