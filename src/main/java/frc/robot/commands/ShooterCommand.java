package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Outtake.shooter;
import frc.robot.subsystems.Turret.Turret;

public class ShooterCommand extends Command{
    private Pose2d pose;
    private shooter shooter;
    
        public ShooterCommand(shooter outtake, Pose2d pose){
            this.pose=pose;
            this.shooter=outtake;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.launcher(this.pose.getY() * 50);
    }

    @Override
    public void end(boolean interrupted){
        shooter.launcher(0.0);
    }
}
