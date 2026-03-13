package frc.robot.commands.TurretTCs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;
import java.util.Optional;

public class AutoAlignTurret extends Command 
{
    private final Turret turret;
    private final Shooter shooter;
    private final PIDController pidController;
    private final Joystick joystick;

    public AutoAlignTurret(Turret turret, Shooter shooter, Joystick joystick) 
    {
        this.turret = turret;
        this.joystick = joystick;
        this.shooter = shooter;

        // idk the pids so something something goes here
        this.pidController = new PIDController(0.015, 0.0, 0.001); 
        
        addRequirements(turret);
    }

    @Override
    public void initialize() 
    {
        LimelightHelpers.setPipelineIndex("limelight", 8);
    }

    @Override
    public void execute() 
    {
        if (joystick.getPOV() == 180) {
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        // long tid = LimelightHelpers.getTV("limelight") == 1 ? LimelightHelpers.getTid("limelight") : -1;
        
        if (7 < Math.abs(tx)){
            turret.setPower(tx * .03);
        }else if (4 < Math.abs(tx)){
            turret.setPower(tx * .01);
        }
        else{
            turret.setPower(0.0);
        }

        shooter.setShooterAngularVelocity((4 - ty / 3.3) / 4 - .13);
    }
        
    }

    private boolean isTargetValidForAlliance(long tid) 
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) 
        {
            if (alliance.get() == Alliance.Red) 
            {
                return tid == 9 || tid == 10;
            } 
            else if (alliance.get() == Alliance.Blue) 
            {
                return tid == 25 || tid == 26;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        turret.setPower(0.0);
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
