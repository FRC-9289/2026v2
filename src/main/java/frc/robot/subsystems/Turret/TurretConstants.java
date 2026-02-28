package frc.robot.subsystems.Turret;

import edu.wpi.first.math.util.Units;

public class TurretConstants
{
    public static final int LAUNCHER_MOTOR_ID_1 = -1;
    public static final int LAUNCHER_MOTOR_ID_2 = -1;

    public static final double CHANGE_IN_HEIGHT = 149.86; // in cm


    public static final double HUB_X = Units.inchesToMeters(182.11); // TODO: measure this (in inches)
    public static final double HUB_Y = Units.inchesToMeters(158.84); // TODO: measure this (in inches)

    public static final double GEAR_RATIO = 5*(200.0 / 25.0); // prevents any integer division errors by casting to double
    public static final double J_TURRET = (1.0/2.0)*(3.3)*(8.5*8.5+12.5*12.5);

    public static final double KS = 0; // Static friction constant

    public static final int MOTOR_ID = 27;
    public static final boolean IS_INVERTED =false;
    public static final double MAX_VEL = Math.toRadians(720.0);
    public static final double MAX_ACCEL = Math.toRadians(720.0);

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double limitCW = Math.PI; // converting values to radian values for softLimits to work
    public static final double limitCCW = Math.PI;

}
