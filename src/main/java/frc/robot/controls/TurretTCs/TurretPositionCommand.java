package frc.robot.controls.TurretTCs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.utils.Constants.TurretConstants;

public class TurretPositionCommand extends Command {

  private final Turret turret;
  private final double targetAngleRad;
  private static final double TOLERANCE = Math.toRadians(1.0);

  public TurretPositionCommand(Turret turret, double targetAngleRad) {
    this.turret = turret;
    this.targetAngleRad = targetAngleRad;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.setDesiredAngle(targetAngleRad);
  }

  @Override
  public boolean isFinished() {
    double error = targetAngleRad - turret.getHeadingRadians();

    // wrap error to (-π, π]
    error = Math.atan2(Math.sin(error), Math.cos(error));

    return Math.abs(error) < TOLERANCE;
  }

  @Override
  public void end(boolean interrupted) {
    turret.setDesiredVelocity(0.0);
  }
}
