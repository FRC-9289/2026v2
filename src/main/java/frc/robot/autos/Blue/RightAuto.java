package frc.robot.autos.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Outtake.CarrierCommand;
import frc.robot.commands.Shooter.ShooterVelocity;
import frc.robot.commands.TurretTCs.SetTurretPosition;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class RightAuto extends SequentialCommandGroup{
    public RightAuto(Shooter shooter, Outtake outtake, Turret turret){
        // addCommands(
        //     new SetTurretPosition(turret, -1.8, false),
        //     new ParallelCommandGroup(
        //     new ShooterVelocity(shooter, 0.7).withTimeout(12),
        //     new ParallelCommandGroup(
        //         new WaitCommand(4),
        //         new CarrierCommand(outtake, () -> 1).withTimeout(8)
        //     )
        //     )
        // );
    }
}
