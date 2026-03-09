package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang.Hang;
import frc.robot.subsystems.Hang.HangConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HangCommand extends Command {

    private final Hang hang;
    private BooleanSupplier goUp;
    private DoubleSupplier goDown;

    private double targetPos = 0.0;
    private boolean lastButtonState = false;
    private boolean currentState = false;

    public HangCommand(Hang hang, DoubleSupplier goDown, BooleanSupplier goUp) {
        this.hang = hang;
        this.goUp = goUp;
        this.goDown = goDown;

        addRequirements(hang);
    }

    @Override
public void execute() {
    if(goUp.getAsBoolean()){
        hang.moveToPos(-143);
    }
    else if(goDown.getAsDouble()>0){
        hang.moveToPos(0.0);
    }
}

    @Override
    public void end(boolean interrupted){
        hang.runTest(0);
    }
}