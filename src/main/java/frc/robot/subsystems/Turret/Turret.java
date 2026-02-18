package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.WolfSparkMax;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Turret extends SubsystemBase {

  private final WolfSparkMax motor;
  private final RelativeEncoder encoder;

  private static final Turret instance = new Turret();

  public static Turret getInstance() {
    return instance;
  }

  public Turret() {
    motor = new WolfSparkMax(
        TurretConstants.MOTOR_ID,
        true,
        false
    );

    encoder = motor.getEncoder();

    resetHeading();
  }

public double getAbsoluteHeadingRadians() {
  double motorRotations = encoder.getPosition();
  double turretRotations = motorRotations / TurretConstants.GEAR_RATIO;
  return turretRotations * 2.0 * Math.PI;
}

  public double getAngularVelocityRadPerSec() {
    double rpm = encoder.getVelocity() / TurretConstants.GEAR_RATIO;
    return rpm * 2.0 * Math.PI / 60.0;
  }

  public void resetHeading() {
    encoder.setPosition(0.0);
  }

public void setDesiredAngle(double angleRad) {

    double current = getAbsoluteHeadingRadians();

    double[] candidates = {
        angleRad,
        angleRad + 2.0 * Math.PI,
        angleRad - 2.0 * Math.PI
    };

    double bestTarget = current;
    double smallestError = Double.POSITIVE_INFINITY;

    for (double candidate : candidates) {

        if (candidate < TurretConstants.limitCCW ||
            candidate > TurretConstants.limitCW) {
            continue;
        }

        double error = Math.abs(candidate - current);

        if (error < smallestError) {
            smallestError = error;
            bestTarget = candidate;
        }
    }

    double motorRotations =
        bestTarget / (2.0 * Math.PI) * TurretConstants.GEAR_RATIO;

    motor.getClosedLoopController()
        .setSetpoint(motorRotations, ControlType.kPosition);
}



  public void setDesiredVelocity(double velocityRadPerSec) {
    double rpm = velocityRadPerSec * 60.0 / (2.0 * Math.PI);
    double motorRpm = rpm * TurretConstants.GEAR_RATIO;

    motor
      .getClosedLoopController()
      .setSetpoint(motorRpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret-Position", getAbsoluteHeadingRadians());
    SmartDashboard.putNumber("Turret-Velocity", getAngularVelocityRadPerSec());

    Logger.recordOutput("Turret Position (rad)", getAbsoluteHeadingRadians());
    Logger.recordOutput("Turret Velocity (rad/s)", getAngularVelocityRadPerSec());
    Logger.recordOutput("Turret Setpoint", motor.getClosedLoopController().getSetpoint());
  }
}
