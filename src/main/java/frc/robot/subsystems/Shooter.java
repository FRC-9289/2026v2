package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.TurretConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    private final SparkMax launcher1;
    private final SparkMax launcher2;

    public static double calculatedDistance; // needs to be dynamically updated with pose

    private double speedLauncher;
    private double targetVoltage; // store the voltage calculated

    private final SimpleMotorFeedforward launcherFeedForward;

    public Shooter() {
        launcher1 = new SparkMax(Constants.TurretConstants.LAUNCHER_MOTOR_ID_1, MotorType.kBrushless);
        launcher2 = new SparkMax(Constants.TurretConstants.LAUNCHER_MOTOR_ID_2, MotorType.kBrushless);

        launcherFeedForward =
            new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
    }

    public static double calculateVelocityFromDistanceToHub(double distance) { // in cm input, converted to m at end of method
        double numerator = 980.665 * Math.pow(distance, 2);
        double denominator = -2 * TurretConstants.CHANGE_IN_HEIGHT + 4 * distance;
        return Math.sqrt(numerator / denominator) / 100;
    }

    public double velocityToVoltage(double velocity) {
        return launcherFeedForward.calculate(velocity);
    }

    @Override
    public void periodic() {
        speedLauncher = calculateVelocityFromDistanceToHub(calculatedDistance);

        targetVoltage = velocityToVoltage(speedLauncher); // not sending voltage to shooting motors yet

        SmartDashboard.putNumber("Shooter - Calculated Distance", calculatedDistance);
        SmartDashboard.putNumber("Shooter - Speed of Launcher", speedLauncher);
        SmartDashboard.putNumber("Shooter - Target Voltage", targetVoltage);
    }

    public void shoot() {
        launcher1.setVoltage(targetVoltage);
        launcher2.setVoltage(-targetVoltage);
    }

    public void stop() {
        launcher1.stopMotor();
        launcher2.stopMotor();
    }
}