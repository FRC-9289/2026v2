package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {
    private Intake module;
    private BooleanSupplier toggle;
    private DoubleSupplier speedSup;

    private static double armPos = 0;
    private static double storagePos = 0;

    public IntakeCommand(Intake module, BooleanSupplier toggle, DoubleSupplier speedSup) {
        this.module = module;
        this.toggle = toggle;
        this.speedSup = speedSup;
        addRequirements(module);
    }

    @Override
    public void initialize() {
        if (toggle.getAsBoolean()) {
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
        module.roller(1);
    }

    @Override
    public void end(boolean interrupted) {
        module.roller(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
