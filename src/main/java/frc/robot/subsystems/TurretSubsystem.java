package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase
{
    private SparkMax launcher1;
    private SparkMax launcher2;
    private SparkMax turretMotor;

    public static double calculatedDistance;

    public static double speedLauncher;

    public TurretSubsystem() {
        launcher1 = new SparkMax(Constants.TurretConstants.LAUNCHER_MOTOR_ID_1, MotorType.kBrushless);
        launcher2 = new SparkMax(Constants.TurretConstants.LAUNCHER_MOTOR_ID_2, MotorType.kBrushless);
        turretMotor = new SparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
        
        speedLauncher = calculateVelocityFromDistanceToHub(calculatedDistance);
    }

    public void shoot() 
    {
        launcher1.set(speedLauncher); 
        launcher2.set(-speedLauncher);
    }

    public void stop() 
    {
        launcher1.stopMotor();
        launcher2.stopMotor();
        turretMotor.stopMotor();
    }

    public static double calculateVelocityFromDistanceToHub(double distance) // distance in cm, converted to m at the end of the method
    {
        double numerator = 980.665 * Math.pow(distance, 2);
        double denominator = -2 * TurretConstants.CHANGE_IN_HEIGHT + 4 * distance;
        return Math.sqrt(numerator / denominator) / 100;
    }
}
