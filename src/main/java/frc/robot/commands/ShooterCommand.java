package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private BooleanSupplier active;
    private Timer timer;
    private double duration = 3.0; // Duration in seconds for which the shooter should run
    
    public ShooterCommand(Shooter outtake, BooleanSupplier active){
        this.outtake=outtake;
        this.active=active;
        this.timer = new Timer();
        addRequirements(outtake);
    }
    
    @Override
    public void initialize(){
        timer.start();
    }
    
    @Override
    public void execute(){
        Pose2d currentPose = Swerve.getInstance().getPose();
        double distance = currentPose.getTranslation().getDistance(new Translation2d(0,0));
        if (active.getAsBoolean()) {
            outtake.calculateShooterVelocity(distance);
            
        } else {
            outtake.setShooterAngularVelocity(0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        outtake.setShooterAngularVelocity(0.0);
    }
}
