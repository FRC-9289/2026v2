package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private SparkMax intakeMotor1;
    private SparkMax intakeMotor2;

    public Intake() {
        intakeMotor1 = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID_1, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID_2, MotorType.kBrushless);
    }

    public void pullIn() {
        intakeMotor1.set(IntakeConstants.SPEEDF);
        intakeMotor2.set(-IntakeConstants.SPEEDF);
    }

    public void stop() {
        intakeMotor1.stopMotor();
        intakeMotor2.stopMotor();
    }

    private static Intake instance;
    public static Intake getInstance(){
        return instance;
    }
}