package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {
    private Intake module;
    private static double armPos = 0;
    private static double storagePos = 0;
    private double speed = 1;

    public IntakeCommand(Intake module) {
        this.module = module;
        addRequirements(module);
    }

    @Override
    public void execute() {
        module.roller(this.speed);
    }

    @Override
    public boolean isFinished() {
        return module.atSetpoint();
    }
}
//Wolfram121