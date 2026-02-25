package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

import frc.robot.utils.WolfSparkMax;

public class Intake extends SubsystemBase {
    private WolfSparkMax arm;
    private WolfSparkMax roller;
    private WolfSparkMax storage;

    public Intake() {

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
        cfg.encoder.positionConversionFactor(360);

        //arm = new WolfSparkMax(50, true, false);
        //arm.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        roller = new WolfSparkMax(22, false, false);
        roller.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        storage = new WolfSparkMax(IntakeConstants.MOTOR_ID, true, false);
    }

    public void roller(double vel) {
        roller.set(vel);
    }

    public void arm(double pos) {
        this.storage.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -180, 180), ControlType.kPosition);
    }

    public void storage(double pos) {
        this.arm.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -180, 180), ControlType.kPosition);
    }

    public boolean atSetpoint() {
        return arm.getClosedLoopController().isAtSetpoint() && storage.getClosedLoopController().isAtSetpoint();
    }
}
//Wolfram121