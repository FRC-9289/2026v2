package frc.robot.controls.TurretTCs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretVelocityCommand extends Command {
    private final TurretSubsystem turret;
    private final double velocity;

    public TurretVelocityCommand(TurretSubsystem turret, double velocity) {
        this.turret = turret;
        this.velocity = velocity;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setDesiredVelocity(velocity); // rad/s
    }

    @Override
    public void execute() {
        // velocity is continuously applied via subsystem periodic()
    }

    @Override
    public void end(boolean interrupted) {
        turret.setDesiredVelocity(0.0); // stop turret
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted
    }
}

