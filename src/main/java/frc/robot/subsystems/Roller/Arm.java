package frc.robot.subsystems.Roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.WolfSparkMax;

public class Arm extends SubsystemBase {
    private WolfSparkMax arm;
    // private static Arm armInstance;

    // public static Arm getInstance() {
    //     return armInstance;
    // }
    

    public Arm() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.pid(RollerConstants.kP, RollerConstants.kI, RollerConstants.kD);

        arm = new WolfSparkMax(RollerConstants.ARM_MOTOR_ID, true, false);
        arm.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void rotateArm(double pos) {
        // arm.getClosedLoopController().setSetpoint(pos, ControlType.kPosition);
        arm.set(pos);
    }

    public void rotateArmToSetpoint(double pos) {
        double error = arm.getEncoder().getPosition() - pos;
        if(Math.abs(error) < 0.1) {
            arm.set(0);
        } else {
            arm.set(0.1);
        }
    }

    // public void storage(double pos) {
    //     this.arm.getClosedLoopController().setSetpoint(MathUtil.clamp(pos, -180, 180), ControlType.kPosition);
    // }

    // public boolean atSetpoint() {
    //     return arm.getClosedLoopController().isAtSetpoint() && storage.getClosedLoopController().isAtSetpoint();
    // }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Roller Position", arm.getEncoder().getPosition());
    }
}
//Wolfram121