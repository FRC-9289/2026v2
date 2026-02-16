package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.auton.RunTest;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Drivetrain.Swerve;

public class RobotContainer {

    /* Driver joystick */
    private final Joystick driver = new Joystick(0);

    /* Axis mappings */
    private final int translationAxis = 1; // forward/back
    private final int strafeAxis = 0;      // left/right
    private final int rotationAxis = 2;    // twist

    /* Buttons */
    private final JoystickButton zeroGyro =
        new JoystickButton(driver, 1);

    private final JoystickButton robotCentric =
        new JoystickButton(driver, 2);

    /* Subsystem */
    public final Swerve s_Swerve;

    /* Auton chooser */
    private final SendableChooser<Command> autonChooser =
        new SendableChooser<>();

    public RobotContainer() {

        /* Start cameras */
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);

        /* Create drivetrain */
        s_Swerve = new Swerve();

        /* Default drive command */
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis) * 0.3,
                () -> driver.getRawAxis(strafeAxis) * 0.3,
                () -> -driver.getRawAxis(rotationAxis) * 0.2,
                () -> robotCentric.getAsBoolean()
            )
        );

        /* Register PathPlanner stop command */
        NamedCommands.registerCommand(
            "Swerve Stop",
            new SwerveCommand(s_Swerve,new Translation2d(0.0,0.0),0.0,false,false).withTimeout(3)
        );

        /* Setup auton chooser */
        autonChooser.setDefaultOption(
            "RunTest MF1m",
            new RunTest("MF1m")
        );

        autonChooser.addOption(
            "RunTest MF2m",
            new RunTest("MF2m")
        );

        autonChooser.addOption(
            "RunTest R90",
            new RunTest("R90")
        );

        autonChooser.addOption(
            "RunTest R180",
            new RunTest("R180")
        );

        autonChooser.addOption(
            "PathPlanner MidReefAuto",
            new PathPlannerAuto("MidReefAuto")
        );

        SmartDashboard.putData("Auton Chooser", autonChooser);

        configureBindings();
    }

    private void configureBindings() {

        /* Zero gyro */
        zeroGyro.onTrue(
            new InstantCommand(() -> s_Swerve.zeroHeading())
        );
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /* Optional test pose helper */
}
