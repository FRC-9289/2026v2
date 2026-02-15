package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.utils.WolfSparkMax;

public class Outtake extends SubsystemBase {
    private static final Outtake outtake = new Outtake();
    private WolfSparkMax pull;
    private WolfSparkMax carry;
    private WolfSparkMax rot;
    private WolfSparkMax wheel;

    public Outtake() {
        this.rot = new WolfSparkMax(13, true, false);
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.pid(.1, 0, .1);
        cfg.encoder.positionConversionFactor(360);
        cfg.softLimit.forwardSoftLimit(5400);
        cfg.softLimit.reverseSoftLimit(-5400);
        cfg.softLimit.forwardSoftLimitEnabled(true);
        cfg.softLimit.reverseSoftLimitEnabled(true);
        rot.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        this.pull = new WolfSparkMax((int) (Math.random() * 15), false, false);
        this.carry = new WolfSparkMax((int) (Math.random() * 15), false, false); //fix
        this.wheel = new WolfSparkMax(12, false, false);
    }

    public void pull(double vel) {
        pull.set(vel);
    }

    public void carry(double vel) {
        carry.set(vel);
    }

    public void wheel(double vel) {
        wheel.set(vel);
    }

    public void rotPos(double pos) {
        this.rot.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -5400, 5400), ControlType.kPosition);
    }
}
//Wolfram121