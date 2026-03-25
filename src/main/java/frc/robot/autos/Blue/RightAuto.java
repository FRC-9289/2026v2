package frc.robot.autos.Blue;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Intake.IntakeCommandDown;
import frc.robot.commands.Outtake.CarrierCommand;
import frc.robot.commands.Shooter.ShooterVelocity;
import frc.robot.commands.TurretTCs.SetTurretPosition;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Roller.Roller;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class RightAuto extends SequentialCommandGroup {
    public RightAuto(Shooter shooter, Outtake outtake, Turret turret, Arm arm, Roller roller) {

        addCommands(

            // Aim
            new SetTurretPosition(turret, -1.85),

            // Shoot sequence
            new ParallelCommandGroup(
                new ShooterVelocity(shooter, 0.75).withTimeout(12),

                new SequentialCommandGroup(
                    new WaitCommand(2),
                    new ParallelCommandGroup(
                        new CarrierCommand(outtake, () -> 1).withTimeout(8)
                    )
                )
            ),

            // Return to zero
            new SetTurretPosition(turret, 0),

            // Ensure it reached target
            new WaitUntilCommand(() ->
                Math.abs(turret.getHeadingRotations()) < 0.05
            ),

            // Stop control cleanly
            new InstantCommand(turret::turnSetpointOff, turret)
        );
    }
}