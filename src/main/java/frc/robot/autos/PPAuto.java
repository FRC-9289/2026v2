package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PPAuto extends SequentialCommandGroup {
    public PPAuto(String name) {
        addCommands(
            new PathPlannerAuto(name)
        );
    }
}