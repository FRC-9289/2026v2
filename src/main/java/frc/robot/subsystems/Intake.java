package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

import frc.robot.utils.WolfSparkMax;

public class Intake extends SubsystemBase {
    private static final Intake intake = new Intake();
    private WolfSparkMax rot;
    private WolfSparkMax wheel;
    private WolfSparkMax sto;

    public Intake() {

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.pid(.1, 0, .1);
        cfg.encoder.positionConversionFactor(360);

        rot = new WolfSparkMax((int) (Math.random() * 15), true, false);
        rot.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        wheel = new WolfSparkMax((int) (Math.random() * 15), false, false);
        wheel.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        sto = new WolfSparkMax((int) (Math.random() * 15), true, false);
    }
    
    public void rot(double speed) {
        rot.set(speed);
    }

    public void wheel(double speed) {
        wheel.set(speed);
    }

    public void sto(double speed) {
        sto.set(speed);
    }

    public static Intake get() {
        return intake;
    }

    public void rotPos(double pos) {
        this.sto.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -180, 180), ControlType.kPosition);
    }

    public void stoPos(double pos) {
        this.rot.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -180, 180), ControlType.kPosition);
    }

    public boolean atSetpoint() {
        return rot.getClosedLoopController().isAtSetpoint() && sto.getClosedLoopController().isAtSetpoint();
    }
}
//Wolfram121