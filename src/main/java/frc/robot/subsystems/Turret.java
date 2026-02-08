package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.WolfSparkMax;
import frc.robot.utils.Constants.NEOMotorConstants;
import frc.robot.utils.Constants.TurretConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Turret extends SubsystemBase {

  private final WolfSparkMax motor;
  private final RelativeEncoder encoder;

  private final PIDController pid;
  private final SimpleMotorFeedforward feedforward;

  private final TrapezoidProfile profile;
  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State start;
  private TrapezoidProfile.State goal;

  private double timeSeconds = 0.0;
  private static final double DT = 0.02;

  private double desiredVelocity = 0.0; // for direct velocity commands
  private boolean usingVelocity = false; // true if last command was velocity

  private static final Turret instance = new Turret();

  public static Turret getInstance() {
    return instance;
  }

  private Turret() {

    motor = new WolfSparkMax(
        TurretConstants.MOTOR_ID,
        MotorType.kBrushless,
        IdleMode.kBrake,
        NEOMotorConstants.CURRENT_LIMIT,
        TurretConstants.IS_INVERTED
    );

    encoder = motor.getEncoder();

    pid = new PIDController(
        0.0,
        0.0,
        0.0
    );

    feedforward = new SimpleMotorFeedforward(
        TurretConstants.kS,
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
  }

  /** Move turret to a target angle (radians) */
  public void setDesiredAngle(double angleRad) {
    timeSeconds = 0.0;
    usingVelocity = false; // last command is position

    start = new TrapezoidProfile.State(
        getHeadingRadians(),
        getAngularVelocityRadPerSec()
    );

    goal = new TrapezoidProfile.State(angleRad, 0.0);
  }

  /** Spin turret at a constant angular velocity (rad/s) */
  public void setDesiredVelocity(double velocityRadPerSec) {
    desiredVelocity = velocityRadPerSec;
    usingVelocity = true; // last command is velocity

    // optional: reset profile start state to avoid jumps if switching later
    start.velocity = getAngularVelocityRadPerSec();
    start.position = getHeadingRadians();
    timeSeconds = 0.0;
  }

  @Override
  public void periodic() {
    double ffVolts;
    double fbVolts;

    if (!usingVelocity) {
      // POSITION MODE
      timeSeconds += DT;

      TrapezoidProfile.State setpoint =
          profile.calculate(timeSeconds, start, goal);

      double desiredPosition = setpoint.position;
      double desiredVel = setpoint.velocity;

      ffVolts = feedforward.calculate(desiredVel);
      fbVolts = pid.calculate(getHeadingRadians(), desiredPosition);

      SmartDashboard.putNumber("Turret/SetpointPos", desiredPosition);
      SmartDashboard.putNumber("Turret/SetpointVel", desiredVel);

    } else {
      // VELOCITY MODE
      ffVolts = feedforward.calculate(desiredVelocity);
      fbVolts = pid.calculate(getAngularVelocityRadPerSec(), desiredVelocity);

      SmartDashboard.putNumber("Turret/SetpointVel", desiredVelocity);
    }

    double totalVolts = MathUtil.clamp(
        ffVolts + fbVolts,
        -NEOMotorConstants.MAX_VOLTAGE,
        NEOMotorConstants.MAX_VOLTAGE
    );

    motor.setVoltage(totalVolts);

    SmartDashboard.putNumber("Turret/Position", getHeadingRadians());
    SmartDashboard.putNumber("Turret/Velocity", getAngularVelocityRadPerSec());
  }

  public double getHeadingRadians() {
    double rotations = encoder.getPosition() / TurretConstants.GEAR_RATIO;
    return rotations * 2.0 * Math.PI;
  }

  public double getAngularVelocityRadPerSec() {
    double rpm = encoder.getVelocity() / TurretConstants.GEAR_RATIO;
    return rpm * 2.0 * Math.PI / 60.0;
  }
}
