package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class ShooterVelocity extends Command{
    private double speed;
    private Shooter outtake;
    private Joystick d;
    private double duration = 3.0; // Duration in seconds for which the shooter should run
    private double shooter=0.0;
    
    public ShooterVelocity(Shooter outtake, double speed){
        this.outtake=outtake;
        this.speed=speed;
        addRequirements(outtake);
    }
    
    @Override
    public void execute(){
        outtake.setShooterAngularVelocity(speed);
    }

    @Override
    public void end(boolean interrupted){
        outtake.setShooterAngularVelocity(0.0);
    }
}
