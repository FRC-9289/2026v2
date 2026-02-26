package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Roller.Roller;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {
    private final Roller module;
    private final BooleanSupplier intakeButton;

    public IntakeCommand(Roller module, BooleanSupplier intakeButton) {
        this.module = module;
        this.intakeButton = intakeButton;
        addRequirements(module);
    }

    @Override
    public void execute() {
        if (intakeButton.getAsBoolean()) {
            module.roller(-1.0);
        } else {
            module.roller(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        module.roller(0.0);
    }
}