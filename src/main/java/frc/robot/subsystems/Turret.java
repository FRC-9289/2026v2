package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants;
import frc.robot.utils.TurretSparkMax;
import frc.robot.utils.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private TurretSparkMax launcherMotor;
    private TurretSparkMax bottomMotor;
    private RelativeEncoder bottomEncoder;

    private double velocity = 0.0;
    private double angle = 0.0;

    public Turret() {
        launcherMotor = new TurretSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID_LAUNCHER, MotorType.kBrushless);
        bottomMotor = new TurretSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID_BOTTOM, MotorType.kBrushless);
        bottomEncoder = bottomMotor.getEncoder();
    }

    public void launchFuel(int speed) {
        launcherMotor.set(speed);
    }

    public void stopLaunch() {
        launcherMotor.set(0);
    }

    public void turnTurret(double targetAngleRadians)
    {
        double currentAngle = bottomEncoder.getPosition();
        double difference = targetAngleRadians - currentAngle;

        double kP = 5.0;
        double kD = 0.1;
        double desiredVelocity = kP * difference;           
        double desiredAcceleration = kD * desiredVelocity;

        double voltage = calculateVoltage(desiredVelocity, desiredAcceleration);

        bottomMotor.setVoltage(voltage);

        setVoltage(desiredVelocity, desiredAcceleration);
    }

    public double getTurretAngle() 
    {
        return bottomEncoder.getPosition(); // returns in radians
    }

    public void setVoltage(double angularVelocity, double angularAcceleration) {
        double voltage = calculateVoltage(angularVelocity, angularAcceleration);

        double accel = (voltage * bottomMotor.KtNMPerAmp) / TurretConstants.J_TURRET;
        accel = Math.max(-maxAccel(), Math.min(maxAccel(), accel));

        velocity += accel * 0.02; // assuming 20ms loop
        angle += velocity * 0.02;
    }

    private double maxAccel() {
        return (TurretConstants.GEAR_RATIO * bottomMotor.KtNMPerAmp * TurretConstants.MAX_CURRENT)
                / TurretConstants.J_TURRET;
    }

    public double calculateVoltage(double angularVelocity, double angularAcceleration) {
        double ka = (TurretConstants.J_TURRET * TurretConstants.R)
                / (TurretConstants.GEAR_RATIO * TurretConstants.KT)
                * angularAcceleration;
        double kv = (TurretConstants.KE * TurretConstants.GEAR_RATIO) * angularVelocity;
        return ka + kv + TurretConstants.KS;
    }
}
