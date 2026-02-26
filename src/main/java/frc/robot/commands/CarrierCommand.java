package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Turret.Turret;

public class CarrierCommand extends Command{
    private Double speed;
    private Outtake outtake;
    
        public CarrierCommand(Outtake outtake, Double speed){
            this.speed=speed;
            this.outtake=outtake;
        addRequirements(outtake);
    }

    @Override
    public void execute(){
        outtake.setPullRotation(speed);
        outtake.setCarryVelocity(speed);
        
        
    }

    @Override
    public void end(boolean interrupted){
        outtake.setCarryVelocity(0.0);
    }
}

