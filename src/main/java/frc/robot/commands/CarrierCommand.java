package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Turret.Turret;

public class CarrierCommand extends Command{
    private DoubleSupplier speed;
    private Outtake outtake;
    
    public CarrierCommand(Outtake outtake, DoubleSupplier speed){
        this.speed=speed;
        this.outtake=outtake;
        addRequirements(outtake);
    }

    @Override
    public void execute(){
        outtake.setPullRotation(speed.getAsDouble());
        outtake.setCarryVelocity(speed.getAsDouble());
        
        
    }

    @Override
    public void end(boolean interrupted){
        outtake.setCarryVelocity(0.0);
    }
}

