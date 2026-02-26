package frc.robot.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunTest extends SequentialCommandGroup{
    public RunTest(String pathName) {
        addCommands(
            new PathPlannerAuto(pathName)
        );
    }
}
