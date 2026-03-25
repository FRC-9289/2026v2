package frc.robot.autos.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Outtake.CarrierCommand;
import frc.robot.commands.Shooter.ShooterVelocity;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class MiddleAuto extends SequentialCommandGroup{
    public MiddleAuto(Shooter shooter, Outtake outtake, Turret turret){
        addCommands(
            new ParallelCommandGroup(
            new ShooterVelocity(shooter, 0.53).withTimeout(12),
            new SequentialCommandGroup(
                new WaitCommand(2.5),
                new CarrierCommand(outtake, () -> 1).withTimeout(8)
            )
            )
        );
    }
}
