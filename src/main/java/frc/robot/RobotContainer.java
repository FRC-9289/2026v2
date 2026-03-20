package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Roller.Roller;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Hang.Hang;
import frc.robot.utils.Constants;
import frc.auton.RunTest;
import frc.robot.autos.PPAuto;
import frc.robot.autos.Blue.*;
import frc.robot.commands.*;
import frc.robot.commands.Hang.HangCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Outtake.CarrierCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Swerve.SetInitialPose;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.TurretTCs.AutoAlignTurret;


public class RobotContainer 
{
    
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Subsystems */
    public Swerve swerve;
    public Outtake outtake;
    public Shooter shooter;
    public Intake intake;
    public Hang hang;
    public Turret turret;
    public Roller roller;
    public Arm arm;

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
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
        turret=new Turret();
        arm = new Arm();
        outtake = new Outtake();
        shooter = new Shooter();
        roller = new Roller();
        hang = new Hang();
        configureButtonBindings();
        configureAutons();
    }

    private void configureButtonBindings() 
    {
        /* Driver Buttons */

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -driver.getRawAxis(1)*0.4, 
                () -> driver.getRawAxis(0) * .4, 
                () -> -driver.getRawAxis(4) * .1, 
                () -> false
            )
        );

        outtake.setDefaultCommand(
            new CarrierCommand(
                outtake, () -> driver.getRawAxis(2)
            )
        );

        Trigger trigger3 = new Trigger(() -> driver.getRawAxis(3)>0);
        trigger3.onTrue(new IntakeCommand(arm, roller, trigger3));
        trigger3.onFalse(new IntakeCommand(arm, roller, trigger3));



        // turret.setDefaultCommand(
        //     new TurretCommand(turret, () -> driver.getRawButton(5), () -> driver.getRawButton(6))
        // );

        hang.setDefaultCommand(
            new HangCommand(hang,
            () -> driver.getRawButton(5), 
            () -> driver.getRawButton(3),
            turret)
        );

        shooter.setDefaultCommand(
            new ShooterCommand(shooter, driver)
        );

        Trigger trigger = new Trigger(() -> driver.getRawButton(10));
        trigger.onTrue(
            new SetInitialPose(swerve, turret)
        );

        // zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    }

    private void configureAutons(){
        autonChooser.setDefaultOption("Do Nothing", new InstantCommand());
        autonChooser.addOption("MF1M", new PPAuto("MF1m"));
        autonChooser.addOption("R180", new PPAuto("R180"));
        autonChooser.addOption("BlueL", new LeftAuto(shooter, outtake, turret));
        autonChooser.addOption("BlueR", new RightAuto(shooter, outtake, turret));
        SmartDashboard.putData("Auton: ", autonChooser);
    }
    
    public Command getAutonomousCommand() 
    {
        // An ExampleCommand will run in autonomous
        return autonChooser.getSelected();
    }

    public Swerve getSwerve() 
    {
        return swerve;
    }
}

