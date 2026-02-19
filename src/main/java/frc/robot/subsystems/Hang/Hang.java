package frc.robot.subsystems.Hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import frc.robot.utils.WolfSparkMax;

public class Hang extends SubsystemBase {

    private WolfSparkMax hangMotor;

    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 10.0;

    public Hang() {

        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.closedLoop.pid(0.1, 0.0, 0.1);

        cfg.softLimit.forwardSoftLimit(MAX_POSITION);
        cfg.softLimit.reverseSoftLimit(MIN_POSITION);
        cfg.softLimit.forwardSoftLimitEnabled(true);
        cfg.softLimit.reverseSoftLimitEnabled(true);

        hangMotor = new WolfSparkMax(10, true, false);

        hangMotor.configure(
            cfg,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void move(double pos) {
        double clampedPos = MathUtil.clamp(pos, MIN_POSITION, MAX_POSITION);

        hangMotor
            .getClosedLoopController()
            .setSetpoint(clampedPos, ControlType.kPosition);
    }

    public boolean atSetpoint() {
        return hangMotor.isAtSetpoint();
    }
}