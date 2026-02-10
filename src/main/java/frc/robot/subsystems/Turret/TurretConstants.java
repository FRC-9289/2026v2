package frc.robot.subsystems.Turret;
import frc.robot.utils.Constants.NEOMotorConstants;

public class TurretConstants
{
    public static final int LAUNCHER_MOTOR_ID_1 = -1;
    public static final int LAUNCHER_MOTOR_ID_2 = -1;
    public static final int TURRET_MOTOR_ID = -1;

    public static final double CHANGE_IN_HEIGHT = 149.86; // in cm


    public static final double HUB_X = 182.11; // in inches
    public static final double HUB_Y = 158.84; // in inches

    public static final double GEAR_RATIO = 5*(200/25);
    public static final double J_TURRET = (1.0/2.0)*(0.200)*(8.5*8.5+12.5*12.5);

    public static final double KS = 0; // Static friction constant

    public static final int MOTOR_ID = 13;
    public static final boolean IS_INVERTED =false;
    public static final double MAX_VEL = Math.toRadians(720.0);
    public static final double MAX_ACCEL = Math.toRadians(720.0);

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double angleOffset = 1.381944; //rad

}
