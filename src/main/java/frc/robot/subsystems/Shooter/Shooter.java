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
import frc.robot.utils.ShooterMath;
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

    public void autoShoot() {
        Pose2d pose = Swerve.getInstance().getPose();
        Pose2d initialPose = Swerve.getInstance().getInitialPose();
        double distance = pose.getTranslation().getDistance(initialPose.getTranslation());
        double velocity = ShooterMath.calculateAngularVelocityFromDistanceToHub(distance, 2.65);
        setShooterAngularVelocity(velocity);
    }

    @Override
    public void periodic() {
        Pose2d pose = Swerve.getInstance().getPose();
        Pose2d initialPose = Swerve.getInstance().getInitialPose();
        SmartDashboard.putNumber("Shooter Velocity", Units.rotationsPerMinuteToRadiansPerSecond(Math.abs(launcher2.getEncoder().getVelocity())));
        double distance = pose.getTranslation().getDistance(initialPose.getTranslation());
        double velocity = ShooterMath.calculateAngularVelocityFromDistanceToHub(distance, 2.65);
        SmartDashboard.putNumber("Calculated Shooter Velocity", velocity);
    }
}
//Wolfram121