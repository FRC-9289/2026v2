package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang.Hang;
import frc.robot.subsystems.Hang.HangConstants;

import java.util.function.BooleanSupplier;

public class HangCommand extends Command {

    private final Hang hang;
    private final BooleanSupplier toggleButton;

    private double targetPos = 0.0;
    private boolean lastButtonState = false;
    private boolean currentState = false;

    public HangCommand(Hang hang, BooleanSupplier toggleButton) {
        this.hang = hang;
        this.toggleButton = toggleButton;

        addRequirements(hang);
    }

    @Override
public void execute() {

    boolean pressed = toggleButton.getAsBoolean();

    if (pressed && !lastButtonState) {
        currentState = !currentState;
    }

    lastButtonState = pressed;

    targetPos = currentState ? HangConstants.MAX_POSITION : HangConstants.MIN_POSITION;

    hang.move(targetPos);
}

    @Override
    public boolean isFinished() {
        return hang.atSetpoint();
    }
}