package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.WolfSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.utils.Constants.TurretConstants;
import java.lang.Math;


/**
 * This class represents the Turret Mechanism, key features:
 * Set to desired angular velocity with feedforward voltage + feedback voltage (rad/s)
 * Set to desired angle (rad)
 */

public class TurretSubsystem extends SubsystemBase {

  // motor
  private WolfSparkMax motor;
  private RelativeEncoder encoder;

  // feedforward
  private SimpleMotorFeedforward ff;

  // PD Controller
  private double kP=0;
  private double kD = 0;
  private PIDController pid;
  private double desiredAngularVelocity;
  private double desiredHeading = 0.0; // rad
  private static final TurretSubsystem turretSubsystem = new TurretSubsystem();

  public TurretSubsystem(){

    motor = new WolfSparkMax(
      TurretConstants.MOTOR_ID, 
      null, 
      null, 
      0, 
      TurretConstants.IS_INVERTED
    );
    encoder = motor.getEncoder();


    // feedforward for calculating voltage
    ff = new SimpleMotorFeedforward(
      TurretConstants.KS, // kS
      TurretConstants.KE * TurretConstants.GEAR_RATIO, //kV
      (TurretConstants.J_TURRET * TurretConstants.R)/ (TurretConstants.GEAR_RATIO * TurretConstants.KT) //kA
    );

    pid = new PIDController(kP, 0, kD);

    desiredAngularVelocity = 0.0;

  }

  /**
   * Set desired angular velocity of turret
   * @param velocity
   */
  public void setDesiredVelocity(double velocity){
    this.desiredAngularVelocity = velocity;
  }

  /**
   * Convert RPM to rad/s
   * @param rpm
   * @return
   */
  public double RPMToRadS(double rpm){
    return rpm * 2*Math.PI/60;
  }

  public double rotationUnitsToRad(double rotationUnits){
    return rotationUnits*2*Math.PI;
  }

  /**
   * Returns velocity of turret (rad/s)
   * @return double turret angular velocity
   */
  public double getAngularVelocityOfTurret(){
    double turret_rpm = -encoder.getVelocity()/TurretConstants.GEAR_RATIO; // Get RPM of motor axle, and divide by gear_ratio to get turret velocity
    double turret_rads = RPMToRadS(turret_rpm); // convert from rpm to rad/s
    return turret_rads;
  }

  public double getHeadingOfTurret(){
    double turret_heading = -encoder.getPosition()/TurretConstants.GEAR_RATIO; // Get rotation units [0-1.0] and divide by gear_ratio to get turret rotation units
    double turret_rad = rotationUnitsToRad(turret_heading);
    return turret_rad;
  }

  /**
   * Calculate required voltage to move turret to a specific velocity
   * @param desiredAngularVelocity
   * @return double
   */
  public double calculateVoltageFeedForward(double desiredAngularVelocity) {
    return ff.calculateWithVelocities(
      getAngularVelocityOfTurret(), 
      desiredAngularVelocity
      );
  }
  /**
   * return PID-based feedback
   * @param angularVelocity
   * @return
   */
  public double calculateVoltageFeedback(double angularVelocity){
    return pid.calculate(getAngularVelocityOfTurret(), angularVelocity);
  }

  @Override
  public void periodic(){
    double ff_volts = calculateVoltageFeedForward(this.desiredAngularVelocity);
    double fb_volts = calculateVoltageFeedback(this.desiredAngularVelocity);

    double total_volts = MathUtil.clamp(
    ff_volts+fb_volts, /* Total voltage */
    -TurretConstants.MAX_VOLTAGE, 
    TurretConstants.MAX_VOLTAGE
    );

    SmartDashboard.putNumber("Turret Angle (rad)", getHeadingOfTurret());
    SmartDashboard.putNumber("Turret Velocity (rad/s)", getAngularVelocityOfTurret());
    SmartDashboard.putNumber("Turret Desired Velocity (rad/s)", desiredAngularVelocity);

    motor.setVoltage(total_volts);
  }

  public static TurretSubsystem getInstance(){
    return turretSubsystem;
  }

  }