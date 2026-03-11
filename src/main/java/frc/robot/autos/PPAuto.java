package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Outtake.CarrierCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Shooter.ShooterVelocity;
import frc.robot.commands.Swerve.SwerveCommand;
import frc.robot.commands.TurretTCs.TurretCommand;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;

public class PPAuto extends ParallelCommandGroup {
    public PPAuto(Shooter shooter, Outtake outtake) {
        addCommands(
            new SequentialCommandGroup(
                new ShooterVelocity(shooter, 0.7).withTimeout(9),
                new WaitCommand(1)
            ),
            new CarrierCommand(outtake, () -> 1.0).withTimeout(7)
        );
    }
}