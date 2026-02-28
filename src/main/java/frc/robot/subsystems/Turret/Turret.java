package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.utils.TurretMath;
import frc.robot.utils.WolfSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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
        true,
        false);

    SparkMaxConfig config = new SparkMaxConfig();
    config.softLimit.forwardSoftLimit(TurretConstants.limitCW);
    config.softLimit.reverseSoftLimit(TurretConstants.limitCCW);

    // sets limits on rotation distance, not angle, as sparkmaxes with encoder.getPosition() return rotations instead
    double forwardLimitRot = TurretConstants.limitCW / (2.0 * Math.PI) * TurretConstants.GEAR_RATIO;
    double reverseLimitRot = TurretConstants.limitCCW / (2.0 * Math.PI) * TurretConstants.GEAR_RATIO;
    config.softLimit.forwardSoftLimit(forwardLimitRot);
    config.softLimit.reverseSoftLimit(reverseLimitRot);

    config.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    resetHeading();
  }

  public double getAbsoluteHeadingRadians() {
    double motorRotations = encoder.getPosition();
    double turretRotations = motorRotations / TurretConstants.GEAR_RATIO;
    return turretRotations * 2.0 * Math.PI; // keeps turret angle as continuous
  }

  public double getAngularVelocityRadPerSec() {
    double rpm = encoder.getVelocity() / TurretConstants.GEAR_RATIO;
    return rpm * 2.0 * Math.PI / 60.0;
  }

  public void resetHeading() {
    encoder.setPosition(0.0);
  }

  public void setDesiredAngle(double targetRad) {

    double current = getAbsoluteHeadingRadians();

    // Shortest path error - allows turret to turn shortest path to face something,
    // for example 20º CW instead of 340º CCW to face a direction
    double error = MathUtil.angleModulus(targetRad - current);

    // Final continuous target
    double finalTarget = current + error;

    // Convert to motor rotations
    double motorRotations = finalTarget / (2.0 * Math.PI) * TurretConstants.GEAR_RATIO;

    motor.getClosedLoopController()
        .setSetpoint(motorRotations, ControlType.kPosition);
  }

  public void runTest(double speed) {
    motor.set(speed);
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
    Rotation2d angle = TurretMath.getDesiredTurretAngle(Swerve.getInstance().getPose(), new Translation2d(0.0, 0.0));
    // this.setDesiredAngle(angle);
    SmartDashboard.putNumber("Angle 2 Hub", angle.getRadians());

    SmartDashboard.putNumber("Turret-Position", getAbsoluteHeadingRadians());
    SmartDashboard.putNumber("Turret-Velocity", getAngularVelocityRadPerSec());
  }
}
