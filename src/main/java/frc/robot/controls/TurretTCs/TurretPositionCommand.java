package frc.robot.controls.TurretTCs;

import org.littletonrobotics.junction.Logger;

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
  }

  @Override
  public void execute() {
    turret.setDesiredAngle(targetAngleRad);
    Logger.recordOutput("Setpoint Angle (rad)", targetAngleRad);
  }

  @Override
  public boolean isFinished() {
    double error = Math.abs(
        turret.getAbsoluteHeadingRadians() - targetAngleRad
    );
    return error < TOLERANCE;
  }

  @Override
  public void end(boolean interrupted) {
    // do nothing, SparkMax holds position
  }
}
