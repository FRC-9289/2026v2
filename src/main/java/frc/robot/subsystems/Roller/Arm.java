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
    private double setpoint;
    // private static Arm armInstance;

    // public static Arm getInstance() {
    //     return armInstance;
    // }
    

    public Arm() {
        SparkMaxConfig cfg = new SparkMaxConfig();

        arm = new WolfSparkMax(RollerConstants.ARM_MOTOR_ID, false, false);
        arm.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        arm.getEncoder().setPosition(0.0);
    }
    public void rotate(double vel) {
        /**
         * Rotate at velocity `vel` between -1 to 1
         */
        arm.set(vel);
    }

    public void setSetpoint(double pos){
        this.setpoint=pos;
    }
    public void rotateArmToSetpoint() {
        /**
         * rotate arm to a setpoint `pos`
         */
        double error = this.setpoint - arm.getEncoder().getPosition();
        double kS = 0.1*Math.signum(error);
        double kV = 0.0*error;
        if(Math.abs(error) > 0.4) {
            arm.set(kS+kV);
        } else {
            arm.set(0.0);
        }
        
        SmartDashboard.putNumber("Arm setpoint", setpoint);
        SmartDashboard.putNumber("Arm Position", arm.getEncoder().getPosition());
    }
}
//Wolfram121