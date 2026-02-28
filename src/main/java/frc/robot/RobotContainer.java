package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Roller.Roller;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Hang.Hang;

import frc.robot.commands.TurretTCs.RunTurretTest;
import frc.auton.RunTest;
import frc.robot.commands.*;


public class RobotContainer {
    
    private final Joystick driver = new Joystick(0);

    /* Subsystems */
    public static Swerve swerve;
    public static Outtake outtake;
    public static Shooter shooter;
    public static Intake intake;
    public static Hang hang;
    public static Turret turret;
    public static Roller roller;
    public static Arm arm;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        String test = "MF1m";
        Pose2d rt;
        //Test poses for auto testing, will be replaced with actual auto paths later
        // switch (test) {
        //     case "MF1m":
        //         rt = new Pose2d(new Translation2d(0.0, 1.0), Rotation2d.fromDegrees(0));
        //         break;
        //     case "MF2m":
        //         rt = new Pose2d(new Translation2d(0.0, 2.0), Rotation2d.fromDegrees(0));
        //         break;
        //     case "R90":
        //         rt = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(90));
        //         break;
        //     case "R180":
        //         rt = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180));
        //         break;
        //     default:
        //         rt = new Pose2d(); // default pose at origin
        // }

        // Initialize drivetrain with target pose
        swerve = Swerve.getInstance();


        //intake.setDefaultCommand(new IntakeCommand(intake, () -> driver.getRawAxis(ControllerConstants.AxisRightTrigger)));
        // outtake = new Outtake();
        // outtake.setDefaultCommand(new OuttakeCommand(outtake, swerve, () -> driver.getRawButton(3)));

        // hang = new Hang();
        // hang.setDefaultCommand(new HangCommand(hang, () -> driver.getRawButton(2)));
        turret=Turret.getInstance();
        arm = Arm.getInstance();
        outtake = Outtake.getInstance();
        shooter = Shooter.getInstance();
        roller = Roller.getInstance();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(1) * -0.3, 
                () -> driver.getRawAxis(0) * .3, 
                () -> driver.getRawAxis(4) * .5, 
                () -> true
            )
        );

        turret.setDefaultCommand(
            new RunTurretTest(
                turret,
                () -> driver.getRawButton(6),
                () -> driver.getRawButton(5)
            )
        );

        
        arm.setDefaultCommand(
            new ArmCommand(
                arm,
                () -> driver.getRawButton(1),
                () -> driver.getRawButton(4)
            )
        );

        outtake.setDefaultCommand(
            new CarrierCommand(
                outtake, () -> driver.getRawAxis(2)
            )
        );

        roller.setDefaultCommand(
            new IntakeCommand(
                roller, 
                () -> driver.getRawButton(3)
            )
        );

        shooter.setDefaultCommand(
            new ShooterCommand(
                shooter, () -> driver.getRawButton(7)
            )
        );

        new JoystickButton(driver, 8).onTrue(
            new SetInitialPose(
                swerve, 
                0, 
                0, 
                Math.toRadians(0), 
                () -> driver.getRawButton(8)
            )
        );

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
