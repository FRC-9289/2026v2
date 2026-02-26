package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.utils.TurretMath;
import frc.robot.utils.WolfSparkMax;

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
      double target = angleRad;

      // Determine direction
      double error = target - current;

      // Prevent moving past CCW limit
      if (error < 0 && current <= TurretConstants.limitCCW) {
          return; // stop, can't go further CCW
      }

      // Prevent moving past CW limit
      if (error > 0 && current >= TurretConstants.limitCW) {
          return; // stop, can't go further CW
      }

      // Command the motor
      double motorRotations = target / (2.0 * Math.PI) * TurretConstants.GEAR_RATIO;
      motor.getClosedLoopController()
          .setSetpoint(motorRotations, ControlType.kPosition);
  }

  public void runTest(double speed){
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
    double angle = TurretMath.getAngleToHub(Swerve.getInstance().getPose());
    // this.setDesiredAngle(angle);
    SmartDashboard.putNumber("Angle 2 Hub", angle);
    SmartDashboard.putNumber("Turret-Position", getAbsoluteHeadingRadians());
    SmartDashboard.putNumber("Turret-Velocity", getAngularVelocityRadPerSec());
  }
}
