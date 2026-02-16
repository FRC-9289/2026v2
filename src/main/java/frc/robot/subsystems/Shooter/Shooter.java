package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.NEOMotorConstants;
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
    private final RelativeEncoder encoder;

    // public static double calculatedDistance; // needs to be dynamically updated
    // with pose

    private double speedLauncher;
    private double distanceMeters;
    private double ffVoltage; // store the voltage calculated
    private double fbVoltage;
    private double targetVoltage;

    private static Shooter instance;
    public static Shooter getInstance(){
        return instance;
    }

    private final SimpleMotorFeedforward launcherFeedForward;
    private final PIDController pid;

    public Shooter(Drivetrain drivetrain) {
        distanceMeters=0;
        launcher1 = new SparkMax(ShooterConstants.LAUNCHER_MOTOR_ID_1, MotorType.kBrushless);

        pid = new PIDController(
                ShooterConstants.SHOOTER_kP,
                ShooterConstants.SHOOTER_kI,
                ShooterConstants.SHOOTER_kD);

        encoder = launcher1.getEncoder();

        launcherFeedForward = new SimpleMotorFeedforward(0, ShooterConstants.kV, ShooterConstants.kA);

        this.drivetrain = Drivetrain.getInstance();
        instance = this;
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
        distanceMeters = robotPose.getTranslation().getDistance(hubPos);

        SmartDashboard.putNumber("Shooter Distance (m)", distanceMeters);
    }

    public void shoot() {
        double ballVelocity = calculateVelocityFromDistanceToHub(distanceMeters);
        double wheelRadPerSec = ballVelocity / ShooterConstants.WHEEL_RADIUS;

        double motorRadPerSec = wheelRadPerSec * TurretConstants.GEAR_RATIO;
        double encoderRadPerSec = encoder.getVelocity() * 2 * Math.PI / 60.0;

        ffVoltage = launcherFeedForward.calculate(motorRadPerSec);
        fbVoltage = pid.calculate(encoderRadPerSec, motorRadPerSec);


        targetVoltage = MathUtil.clamp(
                ffVoltage + fbVoltage,
                -NEOMotorConstants.MAX_VOLTAGE,
                NEOMotorConstants.MAX_VOLTAGE);

        launcher1.setVoltage(targetVoltage);

        SmartDashboard.putNumber("Shooter Ball Vel (m/s)", ballVelocity);
        SmartDashboard.putNumber("Shooter Motor Vel (rad/s)", motorRadPerSec);
        SmartDashboard.putNumber("Shooter Voltage",
        MathUtil.clamp(ffVoltage + fbVoltage, -12, 12));
    }

    public void stop() {
        launcher1.stopMotor();
    }
}