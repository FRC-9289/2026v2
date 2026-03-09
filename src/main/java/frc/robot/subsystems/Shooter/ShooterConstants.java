package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int LAUNCHER_MOTOR_1_ID = 29;
    public static final int LAUNCHER_MOTOR_2_ID = 28;
    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double ROTATOR_RADIUS = Units.inchesToMeters(3); // meters
    public static final double SHOOTER_ANGLE_RAD = Units.degreesToRadians(70);
    public static final double CHANGE_IN_HEIGHT = Units.inchesToMeters(49.69);
}
