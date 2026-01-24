package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.JoystickConstants;
import frc.robot.controls.*;

public class RobotContainer {
  public static final Joystick controller3D = new Joystick(0);
  public static final Joystick wolfByte = new Joystick(1);
  public static double pov;
  public static final JoystickButton resetHeading_Start = new JoystickButton(controller3D, Constants.JoystickConstants.BaseRM);
  
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final SpecDrive specDrive = SpecDrive.getInstance();
  private final Turret turret = Turret.getInstance();
  private final WolfSend wolfSend = WolfSend.getInstance();
  private final WolfPoseEstimator wolfPoseEstimator = WolfPoseEstimator.getInstance();
  
  private ParallelRaceGroup swerveStopCmd;
  SendableChooser<Command> auton_chooser;
  
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0); // Start capturing from the first camera
    CameraServer.startAutomaticCapture(1); // Start capturing from the second camera

    //call to configureBindings() method
    configureBindings();

    // Register swerveStopCmd in Pathplanner to stop robot
    swerveStopCmd = new SwerveDriveCommands(0.0,0.0,0.0).withTimeout(3);
    NamedCommands.registerCommand("Swerve Stop", swerveStopCmd);

    //set up auton commands for the driver
    auton_chooser = new SendableChooser<>();
    auton_chooser.setDefaultOption("MidReefAuto", new PathPlannerAuto("MidReefAuto"));
    SmartDashboard.putData("Auton Chooser", auton_chooser);
    
    // Setup Turret SmartDashboard controls
    setupTurretDashboard();
  }

  private void configureBindings() {
    // Speed multiplier from slider (inverted so up = faster)
    double slider = (-RobotContainer.controller3D.getRawAxis(JoystickConstants.Slider) + 1) / 2.0;
    if (slider == 0)  {
      slider = 0.001;
    }

    // Left joystick - Translation (forward/backward/strafe)
    // Axis 0 = Left Stick X (side/strafe speed)
    // Axis 1 = Left Stick Y (front speed)
    double sideSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.X) * slider;
    double frontSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.Y) * slider;
    
    // Right joystick - Rotation (turning)
    // Axis 2 = Right Stick X / Rotation
    double turnSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.Rot) * slider;

    // Set drivetrain default command with joystick inputs
    drivetrain.setDefaultCommand(new SwerveDriveCommands(frontSpeed, sideSpeed, turnSpeed));

    // Reset heading button (Base RM button)
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

    // POV-based spec drive (comment out before driving, only allows turning)
    pov = wolfByte.getPOV();
    if (pov != -1) {
      specDrive.setDefaultCommand(new SpecDriveCommands(wolfByte.getPOV()));
    }
    
    // Turret control buttons (optional - can be customized)
    JoystickButton turretEnable = new JoystickButton(controller3D, JoystickConstants.RB);
    JoystickButton turretDisable = new JoystickButton(controller3D, JoystickConstants.LB);
    
    turretEnable.onTrue(new InstantCommand(turret::enable, turret));
    turretDisable.onTrue(new InstantCommand(turret::disable, turret));
  }

  private void setupTurretDashboard() {
    // Initialize turret SmartDashboard values
    SmartDashboard.putNumber("Turret/TargetAngle", 0.0);
    SmartDashboard.putNumber("Turret/TargetX", 0.0);
    SmartDashboard.putNumber("Turret/TargetY", 0.0);
    SmartDashboard.putBoolean("Turret/TrackingPoint", false);
    SmartDashboard.putBoolean("Turret/Enabled", true);
    
    // Command buttons for testing
    SmartDashboard.putData("Turret/SetAngle0", new TurretCommands(0.0));
    SmartDashboard.putData("Turret/SetAngle90", new TurretCommands(Math.PI / 2));
    SmartDashboard.putData("Turret/SetAngle180", new TurretCommands(Math.PI));
    SmartDashboard.putData("Turret/SetAngle270", new TurretCommands(3 * Math.PI / 2));
  }

  public Command getAutonomousCommand() {
    return auton_chooser.getSelected();
  }
} //Nice - Wolfram121

