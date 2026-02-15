package frc.robot.subsystems.Hang;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {

    private final SparkMax hangMotor;

    public HangSubsystem() {
        hangMotor = new SparkMax(15, MotorType.kBrushless); 
    }

    public void extend() {
        hangMotor.set(0.9);
    }
    
    public void retract() {
        hangMotor.set(-4.0);
    }
    
    public void stop() {
        hangMotor.set(0);
    }

    private static HangSubsystem instance;
    public static HangSubsystem getInstance(){
        return instance;
    }
}