package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Roller.Roller;

public class RunRoller extends Command{
    private Roller roller;
    private BooleanSupplier b;
    
    public RunRoller(Roller roller, BooleanSupplier b){
        this.roller = roller;
        this.b=b;
    }

    @Override
    public void execute(){
        if(b.getAsBoolean()){
            roller.roller(-1);
        }
        else {
            roller.roller(0);
        }
    }
}
