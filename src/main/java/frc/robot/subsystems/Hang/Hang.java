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

    public Hang() {

        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.closedLoop.pid(HangConstants.kP, HangConstants.kI, HangConstants.kD);

        cfg.softLimit.forwardSoftLimit(HangConstants.MAX_POSITION);
        cfg.softLimit.reverseSoftLimit(HangConstants.MIN_POSITION);
        cfg.softLimit.forwardSoftLimitEnabled(true);
        cfg.softLimit.reverseSoftLimitEnabled(true);

        hangMotor = new WolfSparkMax(HangConstants.MOTOR_ID, true, false);

        hangMotor.getEncoder().setPosition(0);

        hangMotor.configure(
            cfg,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void move(double pos) {
        double clampedPos = MathUtil.clamp(pos,HangConstants.MIN_POSITION, HangConstants.MAX_POSITION);

        hangMotor
            .getClosedLoopController()
            .setSetpoint(clampedPos, ControlType.kPosition);
    }

    public boolean atSetpoint() {
        return hangMotor.getClosedLoopController().isAtSetpoint();
    }
}
