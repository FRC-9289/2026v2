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

    public void launcher(double vel) {
        if (vel < 0){
            launcher1.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
            launcher2.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
        }
        else{
            launcher1.getClosedLoopController().setSetpoint(vel, ControlType.kVelocity);
            launcher2.getClosedLoopController().setSetpoint(-vel, ControlType.kVelocity);
        }
    }

    // public void turret(double pos) {
    //     this.turret.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -5400, 5400), ControlType.kPosition);
    // }
}
//Wolfram121