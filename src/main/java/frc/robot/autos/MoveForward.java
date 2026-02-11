package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Drivetrain.Swerve;

public class MoveForward extends SequentialCommandGroup {

    public MoveForward(Swerve swerve) {
        addCommands(
            new PathPlannerAuto("MF1m")
        );
    }
}