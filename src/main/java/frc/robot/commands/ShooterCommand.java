package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class ShooterCommand extends Command{
    Shooter shooter;
    public ShooterCommand(Shooter shooter) {
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.shoot();
    }
}
