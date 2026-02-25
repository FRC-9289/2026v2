package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Turret.Turret;

public class ArmCommand extends Command{
    private Joystick j;
    private Arm arm;
    
        public ArmCommand(Arm arm, Joystick j){
            this.j=j;
            this.arm=arm;
        addRequirements(arm);
    }

    @Override
    public void execute(){
        if(j.getRawButton(1)){
            arm.arm(0.3);
        }
        else if(j.getRawButton(4)){
            arm.arm(-0.3);
        }
        else{
            arm.arm(0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.arm(0.0);
    }
}
