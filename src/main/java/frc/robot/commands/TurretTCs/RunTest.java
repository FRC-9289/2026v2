package frc.robot.commands.TurretTCs;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class RunTest extends Command{
    private Joystick j;
    private Turret turret;
    
        public RunTest(Turret turret, Joystick j){
            this.j=j;
            this.turret=turret;
        addRequirements(turret);
    }

    @Override
    public void execute(){
        if(j.getRawButton(6)){
            turret.runTest(0.1);
        }
        else if(j.getRawButton(5)){
            turret.runTest(-0.1);
        }
        else{
            turret.runTest(0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        turret.runTest(0.0);
    }
}
