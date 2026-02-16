package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.WolfSparkMax;
import frc.robot.utils.Constants.NEOMotorConstants;

public class Shooter extends SubsystemBase {

  private final Drivetrain drivetrain;

  private final Translation2d hubPos = new Translation2d(8.23, 4.11);

  private final SparkMax launcher1;
  private final RelativeEncoder encoder;

  private static Shooter instance;
  public static Shooter getInstance() {
    return instance;
  }

  public Shooter(Drivetrain drivetrain) {

    launcher1 = new WolfSparkMax(
        ShooterConstants.LAUNCHER_MOTOR_ID_1,
        MotorType.kBrushless,
        IdleMode.kBrake,
        40,
        ShooterConstants.IS_INVERTED
    );
    encoder = launcher1.getEncoder();

    this.drivetrain = drivetrain;
    instance = this;
  }

  public static double calculateVelocityFromDistanceToHub(double distanceMeters) {
    double g = 9.81;
    double theta = ShooterConstants.SHOOTER_ANGLE_RAD;
    double deltaH = ShooterConstants.CHANGE_IN_HEIGHT / 100.0;

    double numerator = g * Math.pow(distanceMeters, 2);
    double denominator = 2 * Math.pow(Math.cos(theta), 2) *
        (distanceMeters * Math.tan(theta) - deltaH);

    return Math.sqrt(numerator / denominator);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = drivetrain.getPose();
    double distanceMeters = robotPose.getTranslation().getDistance(hubPos);

    SmartDashboard.putNumber("Shooter Distance (m)", distanceMeters);
  }

  public void shoot() {

    // compute needed ball velocity
    double distanceMeters =
      drivetrain.getPose()
        .getTranslation()
        .getDistance(hubPos);

    double ballVelocity = calculateVelocityFromDistanceToHub(distanceMeters);

    // convert to motor target rad/sec
    double wheelRadPerSec = ballVelocity / ShooterConstants.WHEEL_RADIUS;
    double motorRadPerSec = wheelRadPerSec * ShooterConstants.GEAR_RATIO;

    // convert to RPM for controller
    double targetRPM = motorRadPerSec * 60.0 / (2.0 * Math.PI);

    // send velocity setpoint to motor controller
    launcher1
      .getClosedLoopController()
      .setSetpoint(targetRPM, ControlType.kVelocity);

    SmartDashboard.putNumber("Shooter Ball Vel (m/s)", ballVelocity);
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    SmartDashboard.putNumber(
        "Shooter Encoder RPM",
        encoder.getVelocity()
    );

    Logger.recordOutput("Shooter/DistanceMeters", distanceMeters);
    Logger.recordOutput("Shooter/BallVelocity", ballVelocity);
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
  }

  public void stop() {
    launcher1.stopMotor();
  }
}
