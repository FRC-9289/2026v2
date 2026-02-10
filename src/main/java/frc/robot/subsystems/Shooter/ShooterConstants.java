package frc.robot.subsystems.Shooter;

public class ShooterConstants {

    public static final int LAUNCHER_MOTOR_ID_1 = -1;
    public static final int LAUNCHER_MOTOR_ID_2 = -1;    
    // PID constants for the launcher
    public static final double SHOOTER_kP = 0.001;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.0;

    // Feedforward constants (tune experimentally)
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.01;

    // Shooter geometry
    public static final double CHANGE_IN_HEIGHT = 0.8; // meters (hub height - shooter exit height)
    public static final double LAUNCH_ANGLE_DEGREES = 45.0; // degrees
    public static final double ROLLER_RADIUS = 0.051; // m
}
