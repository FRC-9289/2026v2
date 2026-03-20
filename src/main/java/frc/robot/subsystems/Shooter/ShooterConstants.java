package frc.robot.subsystems.Shooter;


import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int LAUNCHER_MOTOR_1_ID = 29;
    public static final int LAUNCHER_MOTOR_2_ID = 28;
    public static final double kP = 0.0001;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double G = 9.81;
    public static final double THETA_RAD = Math.toRadians(80);
    public static final double CHANGE_IN_HEIGHT = 1.4;
    public static final double ROTATOR_RADIUS = 0.05;

}
