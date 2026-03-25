package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Roller.Roller;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommandDown extends Command {
    private Arm arm;
    private Roller roller;
    private BooleanSupplier start;

    public IntakeCommandDown(Arm arm, Roller roller, BooleanSupplier start){
        this.arm=arm;
        this.roller=roller;
        this.start=start;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        if(start.getAsBoolean()){
            this.arm.setSetpoint(15);
            arm.rotateArmToSetpoint();
        }
        else {
            arm.rotateArmToSetpoint();
        }
    }

    @Override
    public void end(boolean isInterrupted){
        this.roller.roller(0);
    }
}