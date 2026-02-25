package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Outtake.shooter;
import frc.robot.subsystems.Turret.Turret;

public class ShooterCommand extends Command{
    private DoubleSupplier speed;
    private shooter outtake;
    
        public ShooterCommand(shooter outtake, DoubleSupplier speed){
            this.speed=speed;
            this.outtake=outtake;
        addRequirements(outtake);
    }

    @Override
    public void execute(){
        outtake.launcher(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        outtake.launcher(0.0);
    }
}
