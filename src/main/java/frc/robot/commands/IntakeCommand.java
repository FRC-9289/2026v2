package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Roller.Roller;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {
    private Arm arm;
    private Roller roller;
    private BooleanSupplier start;

    public IntakeCommand(Arm arm, Roller roller, BooleanSupplier start){
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
            this.roller.roller(-1);
            this.arm.setSetpoint(10);
            arm.rotateArmToSetpoint();
        }
        else {
            this.roller.roller(0.0);
            this.arm.setSetpoint(0);
            arm.rotateArmToSetpoint();
        }
    }
}