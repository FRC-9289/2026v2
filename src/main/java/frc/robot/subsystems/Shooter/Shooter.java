package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.utils.WolfSparkMax;

public class Shooter extends SubsystemBase {
    // private WolfSparkMax turret;
    private WolfSparkMax launcher1;
    private WolfSparkMax launcher2;

    private static Shooter shooter=new Shooter();

    public static Shooter getInstance() {
        return shooter;
    }

    public Shooter() {
        shooter = this;
        this.launcher1 = new WolfSparkMax(ShooterConstants.LAUNCHER_MOTOR_1_ID, false, false);
        this.launcher2 = new WolfSparkMax(ShooterConstants.LAUNCHER_MOTOR_2_ID, false, false);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.pid(
            ShooterConstants.kP, 
            ShooterConstants.kI, 
            ShooterConstants.kD
        );
        launcher1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launcher2.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterAngularVelocity(double radPerSec) {
        launcher1.getClosedLoopController().setSetpoint(-radPerSec, ControlType.kVelocity);
        launcher2.getClosedLoopController().setSetpoint(radPerSec, ControlType.kVelocity);
    }

    public void calculateShooterVelocity(double distance) {

        double angularVelocity = calculateAngularVelocityFromDistanceToHub(distance); // Replace with your calculation
        double rpm = Units.radiansPerSecondToRotationsPerMinute(angularVelocity);
        rpm = MathUtil.clamp(rpm*3.0, 0, 5676); //
        setShooterAngularVelocity(rpm);
    }

    public static double calculateAngularVelocityFromDistanceToHub(double distanceMeters) {
        double g = 9.81;
        double theta = ShooterConstants.SHOOTER_ANGLE_RAD;
        double deltaH = ShooterConstants.CHANGE_IN_HEIGHT;

        double numerator = g * Math.pow(distanceMeters, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) *
            (distanceMeters * Math.tan(theta) - deltaH);

        double escapeVelocity = Math.sqrt(numerator / denominator);
        double angularVelocity = escapeVelocity / ShooterConstants.ROTATOR_RADIUS;
        return angularVelocity;
    }

    // public void turret(double pos) {
    //     this.turret.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -5400, 5400), ControlType.kPosition);
    // }
}
//Wolfram121