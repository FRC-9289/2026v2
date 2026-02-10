package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.JoystickConstants;
import frc.robot.controls.*;
import frc.robot.controls.TurretTCs.TurretVelocityCommand;

public class RobotContainer {
  public static final Joystick controller3D = new Joystick(0);
  public static final Joystick wolfByte = new Joystick(1);
  public static double pov;
  public static final JoystickButton resetHeading_Start = new JoystickButton(controller3D, Constants.JoystickConstants.BaseRM);
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Shooter shooter = new Shooter(drivetrain);
  private final SpecDrive specDrive = SpecDrive.getInstance();
  private final Turret turret = Turret.getInstance();
  private final WolfSend wolfSend = WolfSend.getInstance();
  private final WolfPoseEstimator wolfPoseEstimator = WolfPoseEstimator.getInstance();
  private ParallelRaceGroup swerveStopCmd;
  SendableChooser<Command> auton_chooser;

  private int test_number;
  
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
  
    configureBindings();
  
    // swerveStopCmd = new SwerveDriveCommands(0.0,0.0,0.0).withTimeout(3);
    // NamedCommands.registerCommand("Swerve Stop", swerveStopCmd);
  
    // auton_chooser = new SendableChooser<>();
    // auton_chooser.setDefaultOption("MidReefAuto", new PathPlannerAuto("MidReefAuto"));
    // SmartDashboard.putData("Auton Chooser", auton_chooser);
  }

  private void configureBindings() {
    double slider = (-RobotContainer.controller3D.getRawAxis(JoystickConstants.Slider) + 1) / 2.0;
    if (slider == 0)  {
      slider = 0.001;
    }

    double frontSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.X) * slider;
    double sideSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.Y) * slider;
    double turnSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.Rot) * slider;

    // Must rotate CCW and revolve once;

    drivetrain.setDefaultCommand(new SwerveDriveCommands(frontSpeed,sideSpeed,turnSpeed));

    JoystickButton shootButton =
        new JoystickButton(controller3D, 4); // choose button based on driver preference

    shootButton.whileTrue(new RunCommand(shooter::shoot, shooter));
    shootButton.onFalse(new InstantCommand(shooter::stop, shooter));

    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

    //Comment out before driving. Will only let robot turn.
    pov = wolfByte.getPOV();
    if (pov != -1) {
      specDrive.setDefaultCommand(new SpecDriveCommands(wolfByte.getPOV()));
    }
    //specDrive.setDefaultCommand(new SpecDriveCommands2(wolfByte.getRawAxis(0)));
    

    test_number = 1;
  }

  public Command getAutonomousCommand() {
    // return auton_chooser.getSelected();
    switch(test_number){
      case 1:
        return new SequentialCommandGroup(
          new TurretVelocityCommand(turret, Math.PI).withTimeout(4.0)
        );
      case 2:
        return new SequentialCommandGroup(
          new TurretVelocityCommand(turret, Math.PI/4).withTimeout(2.0),
          new TurretVelocityCommand(turret, -Math.PI/4).withTimeout(2.0)
        );
      case 3:
        return new SequentialCommandGroup(
          new TurretVelocityCommand(turret, 0.0).withTimeout(2.0),
          new TurretVelocityCommand(turret, Math.PI).withTimeout(2.0)
        );
      default: return new InstantCommand();
    }
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }
} //Nice - Wolfram121