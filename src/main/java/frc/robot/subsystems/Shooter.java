package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.NEOMotorConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.TurretConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private Drivetrain drivetrain;

    private final Translation2d hubPos = new Translation2d(8.23, 4.11); // in meters, placement of hub from top left
                                                                        // corner of field

    private final SparkMax launcher1;
    private final SparkMax launcher2;
    private final RelativeEncoder encoder;

    // public static double calculatedDistance; // needs to be dynamically updated
    // with pose

    private double speedLauncher;
    private double ffVoltage; // store the voltage calculated
    private double fbVoltage;
    private double targetVoltage;

    private final SimpleMotorFeedforward launcherFeedForward;
    private final PIDController pid;

    public Shooter(Drivetrain drivetrain) {
        launcher1 = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID_1, MotorType.kBrushless);
        launcher2 = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID_2, MotorType.kBrushless);

        pid = new PIDController(
                ShooterConstants.SHOOTER_kP,
                ShooterConstants.SHOOTER_kI,
                ShooterConstants.SHOOTER_kD);

        encoder = launcher1.getEncoder();

        launcherFeedForward = new SimpleMotorFeedforward(0, ShooterConstants.kV, ShooterConstants.kA);

        this.drivetrain = drivetrain;
    }

    public static double calculateVelocityFromDistanceToHub(double distanceMeters) {
        double g = 9.81;
        double theta = ShooterConstants.SHOOTER_ANGLE_RAD; // in radians
        double deltaH = ShooterConstants.CHANGE_IN_HEIGHT / 100; // converted to meters

        double numerator = g * Math.pow(distanceMeters, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) *
                (distanceMeters * Math.tan(theta) - deltaH);

        return Math.sqrt(numerator / denominator);
    }

    @Override
    public void periodic() {
        Pose2d robotPose = drivetrain.getPose();
        double distanceMeters = robotPose.getTranslation().getDistance(hubPos);

        double ballVelocity = calculateVelocityFromDistanceToHub(distanceMeters);
        double wheelRadPerSec = ballVelocity / ShooterConstants.WHEEL_RADIUS;

        double motorRadPerSec = wheelRadPerSec * TurretConstants.GEAR_RATIO;
        double encoderRadPerSec = encoder.getVelocity() * 2 * Math.PI / 60.0;

        ffVoltage = launcherFeedForward.calculate(motorRadPerSec);
        fbVoltage = pid.calculate(encoderRadPerSec, motorRadPerSec);

        SmartDashboard.putNumber("Shooter Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter Ball Vel (m/s)", ballVelocity);
        SmartDashboard.putNumber("Shooter Motor Vel (rad/s)", motorRadPerSec);
        SmartDashboard.putNumber("Shooter Voltage",
                MathUtil.clamp(ffVoltage + fbVoltage, -12, 12));
    }

    public void shoot() {
        targetVoltage = MathUtil.clamp(
                ffVoltage + fbVoltage,
                -NEOMotorConstants.MAX_VOLTAGE,
                NEOMotorConstants.MAX_VOLTAGE);

        launcher1.setVoltage(targetVoltage);
        launcher2.setVoltage(-targetVoltage);
    }

    public void stop() {
        launcher1.stopMotor();
        launcher2.stopMotor();
    }
}