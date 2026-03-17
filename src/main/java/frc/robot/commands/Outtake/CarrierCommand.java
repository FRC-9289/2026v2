package frc.robot.commands.Outtake;

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
        if(speed.getAsDouble()>0){
        outtake.setPullRotation(-1);
        outtake.setCarryVelocity(0.6);
        }
        else{
            outtake.setCarryVelocity(0.0);
            outtake.setPullRotation(0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        outtake.setCarryVelocity(0.0);
    }
}

