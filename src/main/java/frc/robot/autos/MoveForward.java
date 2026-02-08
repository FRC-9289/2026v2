package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SwerveCommand;

public class MoveForward extends SequentialCommandGroup {

    public MoveForward(Swerve swerve) {
        addCommands(
            new SwerveCommand(
                swerve, 
                new Translation2d(3, 0), // 3 m forward, 0 sideways
                0,                        // no rotation
                false,                    // not field-relative
                false                     // closed-loop = false
            ).withTimeout(2),
            new SwerveCommand(swerve,
                new Translation2d(0,3),
                0, 
                false, 
                false).withTimeout(2)          // run for 3 seconds
        );
    }
}