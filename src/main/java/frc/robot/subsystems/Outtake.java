package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.utils.WolfSparkMax;

public class Outtake extends SubsystemBase {
    private static final Outtake outtake = new Outtake();
    private WolfSparkMax rot;
    private WolfSparkMax wheel;
    private WolfSparkMax carry;

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

        this.wheel = new WolfSparkMax(12, false, false);
        this.carry = new WolfSparkMax((int) (Math.random() * 15), false, false); //fix
    }

    public void wheel(double speed) {
        wheel.set(speed);
    }

    public void carry(double speed) {
        carry.set(speed);
    }

    public static Outtake get() {
        return outtake;
    }

    public void rotPos(double pos) {
        this.rot.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -5400, 5400), ControlType.kPosition);
    }
}
//Wolfram121