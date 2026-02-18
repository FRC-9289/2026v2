package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class TurretPositionCommand extends Command{
    private Turret turret;
    private double angleRad;
    
    public TurretPositionCommand(Turret turret, double angleRad) {
        this.turret = turret;
        this.angleRad = angleRad;
        addRequirements(turret);
    }

    @Override
    public void initialize(){
        turret.setDesiredAngle(angleRad);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
