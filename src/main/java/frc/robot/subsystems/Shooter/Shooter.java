package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.Drivetrain.Swerve;

import frc.robot.utils.WolfSparkMax;

public class Shooter extends SubsystemBase {
    // private WolfSparkMax turret;
    private WolfSparkMax launcher1;
    private WolfSparkMax launcher2;

    public Shooter() {
        // this.turret = new WolfSparkMax(26, true, false);
        // SparkMaxConfig cfg = new SparkMaxConfig();
        // cfg.closedLoop.pid(OuttakeConstants.kP, OuttakeConstants.kI, OuttakeConstants.kD);
        // cfg.encoder.positionConversionFactor(360);
        // cfg.softLimit.forwardSoftLimit(5400);
        // cfg.softLimit.reverseSoftLimit(-5400);
        // cfg.softLimit.forwardSoftLimitEnabled(true);
        // cfg.softLimit.reverseSoftLimitEnabled(true);
        // turret.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // this.pull = new WolfSparkMax(OuttakeConstants.PULL_MOTOR_ID, false, false);
        // this.carry = new WolfSparkMax(OuttakeConstants.CARRY_MOTOR_ID, false, false);
        this.launcher1 = new WolfSparkMax(ShooterConstants.LAUNCHER_MOTOR_1_ID, false, false);
        this.launcher2 = new WolfSparkMax(ShooterConstants.LAUNCHER_MOTOR_2_ID, false, false);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        cfg.smartCurrentLimit(100);
        launcher1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launcher2.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterAngularVelocity(double radPerSec) {
        launcher1.set(radPerSec);
        launcher2.set(-radPerSec);
    }

    // public void turret(double pos) {
    //     this.turret.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -5400, 5400), ControlType.kPosition);
    // }

    public void runShooterAtSpeed(double speed) {
        launcher1.set(MathUtil.clamp(speed/5676,-1,1));
        launcher2.set(MathUtil.clamp(-speed/5676,-1,1));
    }



    
    public void autoShoot(double distanceToHub) {
        double velocity = calculateAngularVelocityFromDistanceToHub(distanceToHub, 2.65);
        runShooterAtSpeed(Units.radiansPerSecondToRotationsPerMinute(velocity));
    }

    public static double calculateAngularVelocityFromDistanceToHub(double distanceMeters, double param) {
        double g = 9.81;
        double theta = ShooterConstants.SHOOTER_ANGLE_RAD;
        double deltaH = ShooterConstants.CHANGE_IN_HEIGHT;

        double numerator = g * Math.pow(distanceMeters, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) *
            (distanceMeters * Math.tan(theta) - deltaH);

        double escapeVelocity = Math.sqrt(numerator / denominator);
        double angularVelocity = escapeVelocity / ShooterConstants.ROTATOR_RADIUS;
        return angularVelocity*param;
    }

    @Override
    public void periodic() {
        Pose2d pose = Swerve.getInstance().getPose();
        SmartDashboard.putNumber("Shooter Velocity", Units.rotationsPerMinuteToRadiansPerSecond(Math.abs(launcher2.getEncoder().getVelocity())));
        double distance = pose.getTranslation().getDistance(new Translation2d(0.45, 0.45));
        double velocity = calculateAngularVelocityFromDistanceToHub(distance, 2.65);
        SmartDashboard.putNumber("Calculated Shooter Velocity", velocity);

        final Joystick driver = new Joystick(0);
        Trigger trigger = new Trigger(() -> driver.getRawButton(7));
        SmartDashboard.putNumber("reset button: ", trigger.getAsBoolean() ? 1 : 0);
    }
}
//Wolfram121