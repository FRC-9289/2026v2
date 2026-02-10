package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.NEOMotorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class Shooter extends SubsystemBase {
        private Drivetrain drivetrain = Drivetrain.getInstance();
    
        Pose2d robotPose = drivetrain.getPose();
        private final Translation2d hubPos = new Translation2d(8.23, 4.11); // in meters, placement of hub from top left corner of field
    
        double distanceMeters = robotPose.getTranslation().getDistance(hubPos);
    
        private final SparkMax launcher1;
        private final SparkMax launcher2;
        private final RelativeEncoder encoder;
    
        public static double calculatedDistance; // needs to be dynamically updated with pose
    
        private double speedLauncher;
        private double ffVoltage; // store the voltage calculated
        private double fbVoltage;
        private double targetVoltage;
    
        private final SimpleMotorFeedforward launcherFeedForward;
        private final PIDController pid;
    
        public Shooter(Drivetrain drivetrain) {
            launcher1 = new SparkMax(ShooterConstants.LAUNCHER_MOTOR_ID_1, MotorType.kBrushless);
            launcher2 = new SparkMax(ShooterConstants.LAUNCHER_MOTOR_ID_2, MotorType.kBrushless);
    
            pid = new PIDController(
                    ShooterConstants.SHOOTER_kP,
                    ShooterConstants.SHOOTER_kI,
                    ShooterConstants.SHOOTER_kD);
    
            encoder = launcher1.getEncoder();
    
            launcherFeedForward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    
            this.drivetrain = drivetrain;
    }

    public static double calculateVelocityFromDistanceToHub(double distanceMeters) {
        double theta = Math.toRadians(ShooterConstants.LAUNCH_ANGLE_DEGREES);
        double g = 9.81; // gravity in m/s²
        double deltaY = ShooterConstants.CHANGE_IN_HEIGHT; // height difference in meters

        double numerator = g * Math.pow(distanceMeters, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distanceMeters * Math.tan(theta) - deltaY);

        if (denominator <= 0) {
            // target is too close or too high for this angle, return a fallback
            return 0;
        }

        return Math.sqrt(numerator / denominator);
    }

    public double velocityToVoltage(double velocity) {
        return launcherFeedForward.calculate(velocity);
    }

    @Override
    public void periodic() {
        Pose2d robotPose = drivetrain.getPose();
        double distanceMeters =
            robotPose.getTranslation().getDistance(hubPos);
    
        calculatedDistance = distanceMeters * 100; // convert to cm for method
    
        speedLauncher = calculateVelocityFromDistanceToHub(calculatedDistance);
    
        ffVoltage = velocityToVoltage(speedLauncher);
        fbVoltage = pid.calculate(encoder.getVelocity(), speedLauncher);
    
        SmartDashboard.putNumber("Shooter - Calculated Distance", calculatedDistance);
        SmartDashboard.putNumber("Shooter - Speed of Launcher", speedLauncher);
        SmartDashboard.putNumber("Shooter - Target Voltage",
                MathUtil.clamp(ffVoltage + fbVoltage, -NEOMotorConstants.MAX_VOLTAGE, NEOMotorConstants.MAX_VOLTAGE));
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