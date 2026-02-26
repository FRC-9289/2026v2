package frc.robot.commands.TurretTCs;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class RunTurretTest extends Command{
    private final Turret turret;
    private final BooleanSupplier forward;
    private final BooleanSupplier reverse;

    public RunTurretTest(Turret turret,
                         BooleanSupplier forward,
                         BooleanSupplier reverse) {
        this.turret = turret;
        this.forward = forward;
        this.reverse = reverse;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (forward.getAsBoolean()) {
            turret.runTest(0.1);
        } 
        else if (reverse.getAsBoolean()) {
            turret.runTest(-0.1);
        } 
        else {
            turret.runTest(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
          turret.runTest(0.0);
    }
}
