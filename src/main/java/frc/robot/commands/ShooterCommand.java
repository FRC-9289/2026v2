package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class ShooterCommand extends Command{
    private DoubleSupplier speed;
    private Shooter outtake;
    private Joystick d;
    private double duration = 3.0; // Duration in seconds for which the shooter should run
    private double shooter=0.0;
    
    public ShooterCommand(Shooter outtake, Joystick d){
        this.outtake=outtake;
        this.d=d;
        addRequirements(outtake);
    }
    
    @Override
    public void execute(){
        if(d.getPOV()==0){
            outtake.setShooterAngularVelocity(0.6);
        }
        else if(d.getPOV()==90){
            outtake.setShooterAngularVelocity(0.5);
        }
        else if(d.getPOV()==180){
            outtake.setShooterAngularVelocity(0.4);
        }
        else if(d.getPOV()==270){
            outtake.setShooterAngularVelocity(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        outtake.setShooterAngularVelocity(0.0);
    }
}
