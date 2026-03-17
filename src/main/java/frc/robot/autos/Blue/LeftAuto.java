package frc.robot.autos.Blue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CarrierCommand;
import frc.robot.commands.ShooterVelocity;
import frc.robot.commands.TurretTCs.SetTurretPosition;
import frc.robot.commands.TurretTCs.TurretCommand;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class LeftAuto extends SequentialCommandGroup{
    public LeftAuto(Shooter shooter, Outtake outtake, Turret turret){
        addCommands(
            new SetTurretPosition(turret, 1.8),
            new ParallelCommandGroup(
            new ShooterVelocity(shooter, 342.365/594).withTimeout(10),
            new ParallelCommandGroup(
                new WaitCommand(2),
                new CarrierCommand(outtake, () -> 1).withTimeout(8)
            )
            )
        );
    }
}
