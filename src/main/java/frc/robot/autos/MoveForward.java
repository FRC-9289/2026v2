package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveForward extends SequentialCommandGroup {
    public MoveForward() {
        addCommands(
            new PathPlannerAuto("MoveForward1m", true)
        );
    }
}