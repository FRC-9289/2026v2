package frc.robot.utils;

import com.revrobotics.spark.SparkMax;

public class TurretSparkMax extends SparkMax
{
    public final double KtNMPerAmp = 0.052;
    
    public TurretSparkMax(int deviceId, MotorType m)
    {
        super(deviceId, m);
    }
}
