package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang.Hang;

import java.util.function.BooleanSupplier;

public class HangCommand extends Command {

    private final Hang hang;
    private final BooleanSupplier toggleButton;

    private double targetPos = 0.0;
    private boolean lastButtonState = false;

    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 10.0;

    public HangCommand(Hang hang, BooleanSupplier toggleButton) {
        this.hang = hang;
        this.toggleButton = toggleButton;

        addRequirements(hang);
    }

    @Override
    public void execute() {

        if (toggleButton.getAsBoolean()) {
            currentState = !currentState;
        }

        targetPos = (currentState) ? MAX_POSITION : MIN_POSITION;

        lastButtonState = currentState;

        hang.move(targetPos);
    }

    @Override
    public boolean isFinished() {
        return hang.atSetpoint();
    }
}