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
import frc.robot.controls.TransferCommands.DirectionTransfer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Transfer.Transfer;

public class RobotContainer {
  public static final Joystick controller3D = new Joystick(0);
  public static final Joystick wolfByte = new Joystick(1);
  public static double pov;
  public static final JoystickButton resetHeading_Start = new JoystickButton(controller3D,
      Constants.JoystickConstants.BaseRM);
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final SpecDrive specDrive = SpecDrive.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Kicker kicker = Kicker.getInstance();
  private final Transfer transfer = Transfer.getInstance();
  private final WolfPoseEstimator wolfPoseEstimator = WolfPoseEstimator.getInstance();
  private ParallelRaceGroup swerveStopCmd;
  SendableChooser<Command> auton_chooser;
  private DirectionTransfer direction = DirectionTransfer.FORWARD;

  public RobotContainer() {
    CameraServer.startAutomaticCapture(0); // Start capturing from the first camera
    CameraServer.startAutomaticCapture(1); // Start capturing from the second camera

    // call to configureBindings() method
    configureBindings();

    // Register swerveStopCmd in Pathplanner to stop robot
    swerveStopCmd = new SwerveDriveCommands(0.0, 0.0, 0.0).withTimeout(3);
    NamedCommands.registerCommand("Swerve Stop", swerveStopCmd);

    // set up auton commands for the driver
    auton_chooser = new SendableChooser<>();
    auton_chooser.setDefaultOption("MidReefAuto", new PathPlannerAuto("MidReefAuto"));
    SmartDashboard.putData("Auton Chooser", auton_chooser);
  }

  private void configureBindings() {
    double slider = (-RobotContainer.controller3D.getRawAxis(JoystickConstants.Slider) + 1) / 2.0;
    if (slider == 0) {
      slider = 0.001;
    }

    double frontSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.X) * slider;
    double sideSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.Y) * slider;
    double turnSpeed = RobotContainer.controller3D.getRawAxis(JoystickConstants.Rot) * slider;

    drivetrain.setDefaultCommand(new SwerveDriveCommands(frontSpeed, sideSpeed, turnSpeed));

    JoystickButton transferForwardButton = new JoystickButton(controller3D, Constants.ControllerConstants.ButtonShoulderL);
    JoystickButton transferBackwardButton = new JoystickButton(controller3D, Constants.ControllerConstants.ButtonShoulderR);

    transferForwardButton.onTrue(new InstantCommand(() -> direction = DirectionTransfer.FORWARD));
    transferBackwardButton.onTrue(new InstantCommand(() -> direction = DirectionTransfer.BACKWARD));

    // button to completely gather fuel by enabling all necessary subsystems -
    // hopper, intake, kicker
    JoystickButton gatherButton = new JoystickButton(controller3D, Constants.ControllerConstants.ButtonY);
    gatherButton.toggleOnTrue(new GatherFuelCommand(intake, kicker, transfer, () -> direction));

    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

    // Comment out before driving. Will only let robot turn.
    pov = wolfByte.getPOV();
    if (pov != -1) {
      specDrive.setDefaultCommand(new SpecDriveCommands(wolfByte.getPOV()));
    }
    // specDrive.setDefaultCommand(new SpecDriveCommands2(wolfByte.getRawAxis(0)));
  }

  public Command getAutonomousCommand() {
    return auton_chooser.getSelected();
  }
} // Nice - Wolfram121
