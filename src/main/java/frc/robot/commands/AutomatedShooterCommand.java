package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Shooter.Shooter;

public class AutomatedShooterCommand extends Command {
    // This command will automatically calculate the necessary shooter velocity and turret heading based on the robot's current position on the field, and then set the shooter and turret to those values.
    Shooter shooter;
    public static boolean isAutomatedShooting = false; // Placeholder for the actual condition to trigger automated shooting
    public AutomatedShooterCommand(Shooter shooter, boolean isAutomatedShooting) {
        // add requirements for shooter and turret subsystems
        addRequirements(shooter);
        this.shooter = shooter;
        this.isAutomatedShooting = !isAutomatedShooting;
    }

    @Override
    public void execute() {
        if(isAutomatedShooting){
            double distanceToHub = Swerve.getInstance()
            .getPose()
            .getTranslation()
            .getDistance(new Translation2d(0.45, 0.45));
            shooter.autoShoot(distanceToHub);
        } else {
            shooter.runShooterAtSpeed(0.0);
        }
    }
    
}
