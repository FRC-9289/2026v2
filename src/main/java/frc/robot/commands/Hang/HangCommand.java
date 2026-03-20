package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang.Hang;
import frc.robot.subsystems.Hang.HangConstants;
import frc.robot.subsystems.Turret.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HangCommand extends Command {

    private final Hang hang;
    private BooleanSupplier goUp;
    private BooleanSupplier goDown;

    private double targetPos = 0.0;
    private boolean lastButtonState = false;
    private boolean currentState = false;
    private Turret turret;

    public HangCommand(Hang hang, BooleanSupplier goDown, BooleanSupplier goUp, Turret turret) {
        this.hang = hang;
        this.goUp = goUp;
        this.goDown = goDown;
        this.turret=turret;

        addRequirements(hang);
    }

    @Override
    public void execute() {
        if(goUp.getAsBoolean()){
            hang.moveToPos(192);
            turret.disableTracking();
            turret.setSetpoint(1);
        }
        else if(goDown.getAsBoolean()){
            hang.moveToPos(0.0);
            turret.disableTracking();
            turret.setSetpoint(1);
        }
    }

    @Override
    public void end(boolean interrupted){
        hang.runTest(0);
    }
}