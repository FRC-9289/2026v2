package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveCommand;

public class PPAuto extends SequentialCommandGroup {
    public PPAuto(String pathName) {
        addCommands(
            new PathPlannerAuto(pathName)
        );
    }
}