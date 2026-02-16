package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.Constants.NEOMotorConstants;

public class ShooterConstants {
    public static final int LAUNCHER_MOTOR_ID_1 = 6;

    public static final double CHANGE_IN_HEIGHT = 149.86; // in cm

    public static final double SHOOTER_kP = 0.0;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.0;

    public static final boolean IS_INVERTED = false;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

    public static final double SHOOTER_ANGLE_RAD = 0.349066; // in radians, based on launcher in a box page

    public static final double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double J_SHOOTER = (1.0/2.0)*(0.200)*(WHEEL_RADIUS)*(WHEEL_RADIUS); // moment of inertia of the shooter, based on launcher in a box page

    public static final double kA = (J_SHOOTER * NEOMotorConstants.R)/(NEOMotorConstants.KT*WHEEL_RADIUS*GEAR_RATIO);
    public static final double kV = NEOMotorConstants.KE/WHEEL_RADIUS;
}
