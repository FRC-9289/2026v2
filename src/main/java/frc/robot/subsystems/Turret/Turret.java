package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.WolfSparkMax;

import frc.robot.utils.Constants.NEOMotorConstants;

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

  private Turret() {
    motor = new WolfSparkMax(
        TurretConstants.MOTOR_ID,
        MotorType.kBrushless,
        IdleMode.kBrake,
        40,
        TurretConstants.IS_INVERTED,
        TurretConstants.kP,
        TurretConstants.kI,
        TurretConstants.kD
    );

    encoder = motor.getEncoder();

    resetHeading();
  }

  public double getHeadingRadians() {
    double rotations = encoder.getPosition() / TurretConstants.GEAR_RATIO;
    double angle = rotations * 2.0 * Math.PI;
    angle -= TurretConstants.angleOffset;
    return MathUtil.angleModulus(angle);
  }

  public double getAngularVelocityRadPerSec() {
    double rpm = encoder.getVelocity() / TurretConstants.GEAR_RATIO;
    return rpm * 2.0 * Math.PI / 60.0;
  }

  public void resetHeading() {
    encoder.setPosition(0.0);
  }

  public void setDesiredAngle(double angleRad) {
    double rotations =
        (MathUtil.angleModulus(angleRad) + TurretConstants.angleOffset)
        / (2.0 * Math.PI);

    motor
      .getClosedLoopController()
      .setSetpoint(rotations * TurretConstants.GEAR_RATIO, ControlType.kPosition);
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
    SmartDashboard.putNumber("Turret-Position", getHeadingRadians());
    SmartDashboard.putNumber("Turret-Velocity", getAngularVelocityRadPerSec());
  }
}
