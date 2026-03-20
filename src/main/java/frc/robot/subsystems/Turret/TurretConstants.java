package frc.robot.subsystems.Turret;

import edu.wpi.first.math.util.Units;

public class TurretConstants
{
    public static final int LAUNCHER_MOTOR_ID_1 = -1;
    public static final int LAUNCHER_MOTOR_ID_2 = -1;

    public static final double CHANGE_IN_HEIGHT = 149.86; // in cm


    public static final double RED_HUB_X = Units.inchesToMeters(492.88); // TODO: measure this (in inches)
    public static final double RED_HUB_Y = Units.inchesToMeters(158.84); // TODO: measure this (in inches)

    public static final double BLUE_HUB_X = Units.inchesToMeters(158.34); // TODO: measure this (in inches)
    public static final double BLUE_HUB_Y = Units.inchesToMeters(158.84); // TODO: measure this (in inches)

    public static final double GEAR_RATIO = (200/25);


    public static final int MOTOR_ID = 26;
    public static final boolean IS_INVERTED =false;
    public static final double MAX_VEL = Math.toRadians(720.0);
    public static final double MAX_ACCEL = Math.toRadians(720.0);

    public static final double ERROR_TOLERANCE = 0.04;
    public static final double KS=0.06;
    public static final double KV=0.05;

}
