package frc.robot.subsystems.Outtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.utils.WolfSparkMax;

public class Outtake extends SubsystemBase {
    private static final Outtake outtake = new Outtake();
    private WolfSparkMax pull;
    private WolfSparkMax carry;
    private WolfSparkMax turret;
    private WolfSparkMax launcher;

    public Outtake() {
        this.turret = new WolfSparkMax(13, true, false);
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.pid(.1, 0, .1);
        cfg.encoder.positionConversionFactor(360);
        cfg.softLimit.forwardSoftLimit(5400);
        cfg.softLimit.reverseSoftLimit(-5400);
        cfg.softLimit.forwardSoftLimitEnabled(true);
        cfg.softLimit.reverseSoftLimitEnabled(true);
        turret.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.pull = new WolfSparkMax((int) (Math.random() * 15), false, false);
        this.carry = new WolfSparkMax((int) (Math.random() * 15), false, false); //fix
        this.launcher = new WolfSparkMax(12, false, false);
    }

    public void pull(double vel) {
        pull.set(vel);
    }

    public void carry(double vel) {
        carry.set(vel);
    }

    public void launcher(double vel) {
        launcher.set(vel);
    }

    public void turret(double pos) {
        this.turret.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -5400, 5400), ControlType.kPosition);
    }
}
//Wolfram121