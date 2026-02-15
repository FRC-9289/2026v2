package frc.robot.subsystems.Hopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    private SparkMax intakeMotor;

    public Hopper() {
        intakeMotor = new SparkMax(HopperConstants.HOPPER_MOTOR_ID, MotorType.kBrushless);
    }

    public void pullIn() {
        intakeMotor.set(HopperConstants.SPEEDF);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    private static Hopper instance;
    public static Hopper getInstance(){
        return instance;
    }
}