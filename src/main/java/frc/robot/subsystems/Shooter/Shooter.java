package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.utils.WolfSparkMax;

public class Shooter extends SubsystemBase {
    // private WolfSparkMax turret;
    private WolfSparkMax launcher1;
    private WolfSparkMax launcher2;

    private static Shooter shooter;

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
    }

    // public void pull(double vel) {
    //     pull.set(vel);
    // }

    // public void carry(double vel) {
    //     carry.set(vel);
    // }

    public void setShooterAngularVelocity(double radPerSec) {
        launcher1.set(-radPerSec);
        launcher2.set(radPerSec);
    }

    public void calculateShooterVelocity(double distance) {

        double velocity = 0.0; // Replace with your calculation
        setShooterAngularVelocity(velocity);
    }

    public static Shooter getInstance() {
        return shooter;
    }

    // public void turret(double pos) {
    //     this.turret.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -5400, 5400), ControlType.kPosition);
    // }
}
//Wolfram121