package frc.robot.subsystems.Hang;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
    private final SparkMax hangMotor;

    public HangSubsystem() {
        hangMotor = new SparkMax(15, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        hangMotor.configure(config, (SparkBase.ResetMode) null, (SparkBase.PersistMode) null);
    }

    public void extend() {
        hangMotor.set(0.5);
    }

    public void retract() {
        hangMotor.set(-0.6);
    }

    public void stop() {
        hangMotor.set(0);
    }
}