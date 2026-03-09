package frc.robot.autos.Blue;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.HangCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateArmToSetpoint;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterVelocity;
import frc.robot.subsystems.Hang.Hang;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Roller.Roller;
import frc.robot.subsystems.Shooter.Shooter;

public class Auton3 extends ParallelDeadlineGroup{
    public Auton3(Shooter outtake, Roller roller, Arm arm, Hang hang){
        super(
            new PathPlannerAuto("Auton2"),
            new SequentialCommandGroup(
                        new WaitCommand(2.3),
                        new ParallelCommandGroup(
                            new IntakeCommand(roller, () -> true).withTimeout(2.59),
                            new RotateArmToSetpoint(arm, 1.5).withTimeout(9.91-2.3)
                        )
                    ),
            new SequentialCommandGroup(
                        new WaitCommand(9.21),
                        new ShooterVelocity(outtake, 0.7).withTimeout(2)
                    ),            
            new SequentialCommandGroup(
                        new WaitCommand(11.67),
                        new HangCommand(hang, () -> 1, () -> true)
                    )
        
        );
    }
}
