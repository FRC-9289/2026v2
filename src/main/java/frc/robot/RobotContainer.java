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
import frc.auton.RunTest;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {
    
    private final Joystick driver = new Joystick(0);

    /* Subsystems */
    public static Swerve swerve;
    public static Outtake outtake;
    public static Intake intake;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
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
        swerve.setDefaultCommand(new TeleopSwerve(swerve, () -> -driver.getRawAxis(0) * .3, () -> driver.getRawAxis(1) * .3, () -> -driver.getRawAxis(2) * .2, () -> true));

        intake = new Intake();
        intake.setDefaultCommand(new IntakeCommand(intake, () -> driver.getRawButton(1)));

        outtake = new Outtake();
        outtake.setDefaultCommand(new OuttakeCommand(outtake, swerve, () -> driver.getRawButton(0)));

        configureButtonBindings();
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
