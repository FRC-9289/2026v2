package frc.robot.commands.TurretTCs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Turret.Turret;
import java.util.Optional;

public class AutoAlignTurret extends Command 
{
    private final Turret turret;
    private final PIDController pidController;

    public AutoAlignTurret(Turret turret) 
    {
        this.turret = turret;
        
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
        boolean tv = LimelightHelpers.getTV("limelight");
        double tid = LimelightHelpers.getFiducialID("limelight");
        
        // does the limelight spy with its little eye something that starts with the letter AprilTag?
        if (tv && isTargetValidForAlliance((long)tid)) 
        {
            double tx = LimelightHelpers.getTX("limelight");
            
            // actual turret math
            double steeringAdjust = pidController.calculate(tx, 0.0);
            
            // change this based off our turret velocities
            steeringAdjust = Math.max(-0.4, Math.min(0.4, steeringAdjust));
            
            turret.setPower(steeringAdjust);
        } else 
        {
            // this was an issue we had a lot in ftc, the turret would just start spinning out of control, so this should stop it if we ever lose sight of the tags
            turret.setPower(0.0); 
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
