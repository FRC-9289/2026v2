package frc.robot.controls;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.TurretConstants;

public class TurretSim extends SubsystemBase {

  private double angle = 0;
  private double velocity = 0;

  private static final DCMotor turretNEO = DCMotor.getNEO(1);

  public void setVoltage(double angularVelocity, double angularAcceleration) {
    double voltage = calculateVoltage(angularVelocity, angularAcceleration);

    double accel =
        (voltage * turretNEO.KtNMPerAmp) / TurretConstants.J_TURRET;

    velocity += accel * 0.02;
    angle += velocity * 0.02;
  }

  public double calculateVoltage(double angularVelocity,
                                 double angularAcceleration) {

    double ka =
        (TurretConstants.J_TURRET * TurretConstants.R)
        / (TurretConstants.GEAR_RATIO * TurretConstants.KT)
        * angularAcceleration;

    double kv =
        (TurretConstants.KE * TurretConstants.GEAR_RATIO)
        * angularVelocity;

    return ka + kv + TurretConstants.KS;
  }

  public double getAngle() { return angle; }
  public double getVelocity() { return velocity; }
}