package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {
    private Intake module;
    private static double armPos = 0;
    private static double storagePos = 0;
    private double speed;

    public IntakeCommand(Intake module, BooleanSupplier x, DoubleSupplier y) {
        this.module = module;

        this.speed = y.getAsDouble();

        if (x.getAsBoolean()) {
            if (armPos == 0 && storagePos == 0) {
                armPos = 360;
                storagePos = 720;
            } else {
                armPos = 0;
                storagePos = 0;
            }
        }
    }

    @Override
    public void execute() {
        module.arm(armPos);
        module.storage(storagePos);
        module.roller(this.speed);
    }

    @Override
    public boolean isFinished() {
        return module.atSetpoint();
    }
}
//Wolfram121