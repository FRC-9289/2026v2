package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Roller.Roller;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {
    private final Roller module;
    private final double speed = 1.0;
    private final Joystick d;

    public IntakeCommand(Roller module, Joystick d) {
        this.module = module;
        this.d= d;
        addRequirements(module);
    }

    @Override
    public void execute() {
        if(d.getRawButton(3)){
         module.roller(-.7);

        }
        else{
            module.roller(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        module.roller(0);
    }

    @Override
    public boolean isFinished() {
        return false; // Button will control when it ends
    }
}