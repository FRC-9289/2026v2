package frc.robot.commands.TurretTCs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class SetTurretPosition extends Command {
    private final Turret turret;
    private final double angle;

    public SetTurretPosition(Turret turret, double angle) {
        this.turret = turret;
        this.angle = angle;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.disableTracking();
        turret.setSetpoint(angle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(turret.getHeadingRotations() - angle) < 0.05;
    }
}