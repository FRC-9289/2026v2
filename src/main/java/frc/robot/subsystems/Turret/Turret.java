package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.NEOMotorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Turret extends SubsystemBase {

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  private final PIDController pid;
  private final SimpleMotorFeedforward feedforward;

  private final TrapezoidProfile profile;
  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State start;
  private TrapezoidProfile.State goal;

  private double timeSeconds = 0.0;
  private static final double DT = 0.02;

  private double desiredVelocity = 0.0;
  private boolean usingVelocity = false;

  private static final Turret instance = new Turret();

  public static Turret getInstance() {
    return instance;
  }

  private Turret() {
    motor = new SparkMax(TurretConstants.MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    pid = new PIDController(
        TurretConstants.kP,
        TurretConstants.kI,
        TurretConstants.kD
    );

    feedforward = new SimpleMotorFeedforward(
        TurretConstants.KS,
        NEOMotorConstants.KE * TurretConstants.GEAR_RATIO,
        (TurretConstants.J_TURRET * NEOMotorConstants.R)
            / (TurretConstants.GEAR_RATIO * NEOMotorConstants.KT)
    );

    constraints = new TrapezoidProfile.Constraints(
        TurretConstants.MAX_VEL,
        TurretConstants.MAX_ACCEL
    );

    profile = new TrapezoidProfile(constraints);

    start = new TrapezoidProfile.State(0.0, 0.0);
    goal = new TrapezoidProfile.State(0.0, 0.0);

    // If positive voltage spins CW, invert:
    // motor.setInverted(true);
  }

  /**
   * CCW positive angle
   * (No negative sign here)
   */
public double getHeadingRadians() {
    double rotations = encoder.getPosition() / TurretConstants.GEAR_RATIO;
    double angle = rotations * 2.0 * Math.PI;

    // If encoder is CW-positive, uncomment:
    // angle = -angle;

    angle -= TurretConstants.angleOffset;
    return MathUtil.angleModulus(angle);
}


  public double getAngularVelocityRadPerSec() {
    double rpm = encoder.getVelocity() / TurretConstants.GEAR_RATIO;
    return rpm * 2.0 * Math.PI / 60.0;  // CCW positive
  }

  public void resetHeading(){
    encoder.setPosition(0.0);
  }

  /** Position control using shortest path */
  public void setDesiredAngle(double angleRad) {
    timeSeconds = 0.0;
    usingVelocity = false;

    double current = getHeadingRadians();
    double target = MathUtil.angleModulus(angleRad);

    // shortest path error
    double error = Math.atan2(Math.sin(target - current), Math.cos(target - current));

    start = new TrapezoidProfile.State(current, getAngularVelocityRadPerSec());
    goal = new TrapezoidProfile.State(current + error, 0.0);
  }

  /** Velocity control */
  public void setDesiredVelocity(double velocityRadPerSec) {
    desiredVelocity = velocityRadPerSec;
    usingVelocity = true;

    start.velocity = getAngularVelocityRadPerSec();
    start.position = getHeadingRadians();
    timeSeconds = 0.0;
  }

  @Override
  public void periodic() {
    double ffVolts;
    double fbVolts;

    double desiredPosition = 0.0;
    double desiredVel = 0.0;

    if (!usingVelocity) {
      // POSITION MODE
      timeSeconds += DT;

      TrapezoidProfile.State setpoint = profile.calculate(timeSeconds, start, goal);

      desiredPosition = setpoint.position;
      desiredVel = setpoint.velocity;

      ffVolts = feedforward.calculate(desiredVel);

      // wrap angle error
      double error = Math.atan2(
          Math.sin(desiredPosition - getHeadingRadians()),
          Math.cos(desiredPosition - getHeadingRadians())
      );

      fbVolts = pid.calculate(0, error);  // PID drives error to 0
    } else {
      // VELOCITY MODE
      desiredVel = desiredVelocity;
      ffVolts = feedforward.calculate(desiredVel);
      fbVolts = pid.calculate(getAngularVelocityRadPerSec(), desiredVelocity);
    }

    double totalVolts = MathUtil.clamp(
        ffVolts + fbVolts,
        -NEOMotorConstants.MAX_VOLTAGE,
        NEOMotorConstants.MAX_VOLTAGE
    );

    motor.setVoltage(totalVolts);

    SmartDashboard.putNumber("Turret-SetpointPos", desiredPosition);
    SmartDashboard.putNumber("Turret-SetpointVel", desiredVel);
    SmartDashboard.putNumber("Turret-Position", getHeadingRadians());
    SmartDashboard.putNumber("Turret-Velocity", getAngularVelocityRadPerSec());
  }
}
