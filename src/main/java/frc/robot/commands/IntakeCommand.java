package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.BooleanSupplier;

public class IntakeCommand extends Command {
    private Intake module;
    private static double rotPos = 0;
    private static double stoPos = 0;

    public IntakeCommand(Intake module, BooleanSupplier x) {
        this.module = module;

        if (x.getAsBoolean()) {
            if (rotPos == 0 && stoPos == 0) {
                rotPos = 360;
                stoPos = 720;
            } else {
                rotPos = 0;
                stoPos = 0;
            }
        }
    }

    @Override
    public void execute() {
        module.rotPos(rotPos);
        module.stoPos(stoPos);
    }

    @Override
    public boolean isFinished() {
        return module.atSetpoint();
    }
}
//Wolfram121