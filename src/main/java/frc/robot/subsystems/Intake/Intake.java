package frc.robot.subsystems.Intake;

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

        rot = new WolfSparkMax((int) (Math.random() * 15), true, false); //fix
        rot.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        wheel = new WolfSparkMax((int) (Math.random() * 15), false, false); //fix
        wheel.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        sto = new WolfSparkMax((int) (Math.random() * 15), true, false); //fix
    }
    
    public void rot(double vel) {
        rot.set(vel);
    }

    public void wheel(double vel) {
        wheel.set(vel);
    }

    public void sto(double vel) {
        sto.set(vel);
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