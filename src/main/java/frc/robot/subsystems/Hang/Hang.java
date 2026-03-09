package frc.robot.subsystems.Hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.WolfSparkMax;

public class Hang extends SubsystemBase {

    private WolfSparkMax hangMotor;

    public Hang() {

        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.closedLoop.pid(HangConstants.kP, HangConstants.kI, HangConstants.kD);

        // cfg.softLimit.forwardSoftLimit(HangConstants.MAX_POSITION);
        // cfg.softLimit.reverseSoftLimit(HangConstants.MIN_POSITION);
        // cfg.softLimit.forwardSoftLimitEnabled(true);
        // cfg.softLimit.reverseSoftLimitEnabled(true);

        hangMotor = new WolfSparkMax(HangConstants.MOTOR_ID, true, false);

        hangMotor.getEncoder().setPosition(0);

        hangMotor.configure(
            cfg,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void moveToPos(double setpointRot) {
        hangMotor.getClosedLoopController().setSetpoint(setpointRot, ControlType.kPosition);
    }

    public void runTest(double vel){
        hangMotor.set(vel);
    }

    public boolean atSetpoint() {
        return hangMotor.getClosedLoopController().isAtSetpoint();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hang motor pos", hangMotor.getEncoder().getPosition());
    }
}
