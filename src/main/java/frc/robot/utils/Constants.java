package frc.robot.utils;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
// import HolonomicPathFollowerConfig;
import frc.robot.subsystems.Turret;

public final class Constants {
  public static class ModuleIDs {
  }

  public static class SwerveIDs {
    public static final int LFD = 4;
    public static final int LFT = 1;
    public static final int LBD = 5;
    public static final int LBT = 6;
    public static final int RBD = 8;
    public static final int RBT = 2;
    public static final int RFD = 3;
    public static final int RFT = 7;
    public static final int PIGEON = 15;

    public static final int LF_CAN = 14;
    public static final int LB_CAN = 12;
    public static final int RB_CAN = 11;
    public static final int RF_CAN = 13;
  }

  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OP_CONTROLLER_PORT = 1;
  }

  public static final class SwerveConstants {
    public static final double LF_OFFSET = 0; // change
    public static final double RF_OFFSET = 0; // change
    public static final double LB_OFFSET = 0; // change
    public static final double RB_OFFSET = 0; // change

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = .5;

    public static final double DRIVETRAIN_MAX_SPEED = 3.91;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3 * Math.PI;

    // Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(20.75);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

  }

  public final class ControllerConstants {
    public static final int AxisLeftStickX = 0;
    public static final int AxisLeftStickY = 1;
    public static final int AxisLeftTrigger = 2;
    public static final int AxisRightTrigger = 3;
    public static final int AxisRightStickX = 4;
    public static final int AxisRightStickY = 5;

    // Gamepad Trigger Threshold
    public static final double TriggerThreshold = 0.1;

    // Gamepad POVs
    public static final int PovUp = 0;
    public static final int PovRight = 90;
    public static final int PovDown = 180;
    public static final int PovLeft = 270;

    // Gamepad buttons
    public static final int ButtonA = 1; // Bottom Button
    public static final int ButtonB = 2; // Right Button
    public static final int ButtonX = 3; // Left Button
    public static final int ButtonY = 4; // Top Button
    public static final int ButtonShoulderL = 5;
    public static final int ButtonShoulderR = 6;
    public static final int ButtonBack = 7;
    public static final int ButtonStart = 8;
    public static final int ButtonLeftStick = 9;
    public static final int ButtonRightStick = 10;
  }

  public static final class JoystickConstants {
    public static final int Y = 0;
    public static final int X = 1;
    public static final int Rot = 2;
    public static final int Slider = 3;

    public static final int PovUp = 0;
    public static final int PovRight = 90;
    public static final int PovDown = 180;
    public static final int PovLeft = 270;

    public static final int Trigger = 1;
    public static final int Side = 2;
    public static final int LB = 3;
    public static final int RB = 4;
    public static final int LF = 5;
    public static final int RF = 6;
    public static final int BaseLF = 7;
    public static final int BaseRF = 8;
    public static final int BaseLM = 9;
    public static final int BaseRM = 10;
    public static final int BaseLB = 11;
    public static final int BaseRB = 12;
  }

  public final class TurretConstants
  {
    // Motor and Hardware Constants
    public static final int MOTOR_ID = 10;
    public static final boolean MOTOR_INVERTED = false;
    
    // Target Coordinate to Track (inches)
    // Coordinate system: Red origin (0,0) = left corner of red alliance side
    // When BLUE_ORIGIN = true: (0,0) = left corner of blue alliance side
    public static final double TARGET_X = 158.32; // X coordinate of target point (inches)
    public static final double TARGET_Y = 181.56; // Y coordinate of target point (inches)
    public static final boolean BLUE_ORIGIN = false; // true = blue origin, false = red origin
    
    // Field dimensions (inches) - Standard FRC Field
    public static final double FIELD_LENGTH_X = 651.22; // ~16.46m (54 feet)
    public static final double FIELD_WIDTH_Y = 317.69;  // ~8.02m (26 feet)
    
    // Joystick button to toggle between red/blue origin
    public static final int ORIGIN_TOGGLE_BUTTON = ControllerConstants.ButtonStart; // Start button
    
    // Physical Constants (default values - change for actual robot)
    public static final double J_TURRET = 0.01; // Moment of inertia (kg*m^2)
    public static final double GEAR_RATIO = 100.0; // Gear reduction ratio
    public static final double MAX_CURRENT = 40.0; // Maximum current (amps)
    
    // Electrical Constants (NEO motor defaults)
    public static final double R = 0.098; // Resistance (ohms) - NEO default
    public static final double KT = 0.00269; // Torque constant (N*m/Amp) - NEO default
    public static final double KE = 1.5; // Back EMF constant (V/(rad/s)) - NEO default
    public static final double KS = 0.0; // Static friction compensation (volts)
    
    // Position Control PID Constants
    public static final double KP = 0.5; // Proportional gain
    public static final double KI = 0.0; // Integral gain
    public static final double KD = 0.0; // Derivative gain
    
    // Motion Limits
    public static final double MAX_VELOCITY = 5.0; // Max angular velocity (rad/s)
    public static final double MAX_ACCELERATION = 10.0; // Max angular acceleration (rad/s^2)
    
    // Angle Limits (in radians)
    public static final double MIN_ANGLE = -Math.PI; // Minimum allowed angle
    public static final double MAX_ANGLE = Math.PI; // Maximum allowed angle
    public static final double OFFSET = 0.0; // Zero position offset (radians)
    
    // Conversion factors
    public static final double ENCODER_PCONVERSION = 2 * Math.PI / GEAR_RATIO; // Position conversion
    public static final double ENCODER_VCONVERSION = ENCODER_PCONVERSION / 60.0; // Velocity conversion
    
    // Deadband for reaching target
    public static final double ANGLE_TOLERANCE = 0.05; // Tolerance in radians
  }
}
