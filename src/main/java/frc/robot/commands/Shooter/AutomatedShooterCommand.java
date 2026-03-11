package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.utils.ShooterMath;

public class AutomatedShooterCommand extends Command {
    // This command will automatically calculate the necessary shooter velocity and turret heading based on the robot's current position on the field, and then set the shooter and turret to those values.
    Shooter shooter;
    Outtake outtake;
    public static boolean isAutomatedShooting = false; // Placeholder for the actual condition to trigger automated shooting
    public AutomatedShooterCommand(Shooter shooter, boolean isAutomatedShooting, Outtake outtake) {
        // add requirements for shooter and turret subsystems
        addRequirements(shooter, outtake);
        this.shooter = shooter;
        this.outtake = outtake;
        this.isAutomatedShooting = !isAutomatedShooting;
    }

    @Override
    public void execute() {
        if(isAutomatedShooting){
            shooter.autoShoot();
        } else {
            shooter.runShooterAtSpeed(0.0);
        }
    }
    
}
