package frc.robot.commands.TurretTCs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class SetTurretPosition extends Command{
    private Turret turret;
    private double angle;
    public SetTurretPosition(Turret turret, double angle){
        this.turret=turret;
        this.angle=angle;
    }

    @Override
    public void execute(){
        Turret.disableTracking();
        turret.setDesiredAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(turret.getHeadingRotations()-angle)>0.05;
    }
}
