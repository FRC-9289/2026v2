package frc.robot.utils;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class WolfSparkMax extends SparkMax {
    public WolfSparkMax(int ID, boolean brake, boolean inverted) {
        super(ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        config.smartCurrentLimit(40);
        config.voltageCompensation(12);

        super.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }
}