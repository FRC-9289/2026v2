package frc.robot.commands.TurretTCs;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class TurretVelocityCommand extends Command {

  private final Turret turret;
  private final double targetVelocityRadPerSec;

  public TurretVelocityCommand(Turret turret, double targetVelocityRadPerSec) {
    this.turret = turret;
    this.targetVelocityRadPerSec = targetVelocityRadPerSec;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    turret.setDesiredVelocity(targetVelocityRadPerSec);
    Logger.recordOutput("Setpoint Velocity (rad)", targetVelocityRadPerSec);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setDesiredVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted
  }
}
