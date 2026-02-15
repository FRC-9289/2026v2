package frc.robot.subsystems.Kicker;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {

    private SparkMax intakeMotor;

    public Kicker() {
        intakeMotor = new SparkMax(KickerConstants.KICKER_MOTOR_ID, MotorType.kBrushless);
    }

    public void goUp() {
        intakeMotor.set(KickerConstants.SPEEDF);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    private static Kicker instance;
    public static Kicker getInstance(){
        return instance;
    }
}