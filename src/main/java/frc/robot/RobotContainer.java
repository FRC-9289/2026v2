package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.TurretTCs.TurretPositionCommand;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Outtake.shooter;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Roller.Roller;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Hang.Hang;
import frc.robot.utils.Constants;

import frc.robot.commands.TurretTCs.RunTurretTest;
import frc.auton.RunTest;
import frc.robot.commands.*;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ArmCommand;


public class RobotContainer {
    
    private final Joystick driver = new Joystick(0);

    /* Subsystems */
    public static Swerve swerve;
    public static Outtake outtake;
    public static shooter shoot;
    public static Intake intake;
    public static Hang hang;
    public static Turret turret;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        turret=Turret.getInstance();
        String test = "MF1m";
        Pose2d rt;
        //Test poses for auto testing, will be replaced with actual auto paths later
        switch (test) {
            case "MF1m":
                rt = new Pose2d(new Translation2d(0.0, 1.0), Rotation2d.fromDegrees(0));
                break;
            case "MF2m":
                rt = new Pose2d(new Translation2d(0.0, 2.0), Rotation2d.fromDegrees(0));
                break;
            case "R90":
                rt = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(90));
                break;
            case "R180":
                rt = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180));
                break;
            default:
                rt = new Pose2d(); // default pose at origin
        }

        // Initialize drivetrain with target pose
        swerve = new Swerve();
        swerve.setDefaultCommand(new TeleopSwerve(swerve, () -> -driver.getRawAxis(1) * -0.3, () -> driver.getRawAxis(0) * .3, () -> driver.getRawAxis(4) * .5, () -> true));


        //intake.setDefaultCommand(new IntakeCommand(intake, () -> driver.getRawAxis(ControllerConstants.AxisRightTrigger)));
        // outtake = new Outtake();
        // outtake.setDefaultCommand(new OuttakeCommand(outtake, swerve, () -> driver.getRawButton(3)));

        // hang = new Hang();
        // hang.setDefaultCommand(new HangCommand(hang, () -> driver.getRawButton(2)));

        turret.setDefaultCommand(new RunTurretTest(turret, driver));

        outtake = new Outtake();
        //outtake.setDefaultCommand(new ShooterCommand(outtake, () -> driver.getRawAxis(3)));
        outtake.setDefaultCommand(new CarrierCommand(outtake, () -> driver.getRawAxis(2)));
        //outtake.setDefaultCommand(new PullCommand(outtake, () -> driver.getRawAxis(3)));
        shoot = new shooter();
        shoot.setDefaultCommand(new ShooterCommand(shoot, () -> driver.getRawAxis(3)));

        Roller roller = new Roller();
        roller.setDefaultCommand(new IntakeCommand(roller, driver));

        configureButtonBindings();

        Arm arm = new Arm();
        arm.setDefaultCommand(new ArmCommand(arm, driver));
    }

    private void configureButtonBindings() {
        /* Driver Buttons */

        // zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));
    }
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        String test = "MF1m";
        return new RunTest(test);
    }

    public static Swerve getSwerve() {
        return swerve;
    }
}
