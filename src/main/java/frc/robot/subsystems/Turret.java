package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.utils.WolfSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class Turret extends SubsystemBase {
    // Singleton instance
    private static final Turret instance = new Turret();
    
    public static Turret getInstance() {
        return instance;
    }
    
    // Motor and encoder
    private WolfSparkMax turretMotor;
    
    // PID Controller for position control
    private PIDController pidController;
    
    // Target and state tracking
    private double targetAngle = 0.0; // Target angle in radians
    private double currentAngle = 0.0; // Current angle in radians
    private double motorPosition = 0.0; // Motor encoder position in rotations
    private boolean isEnabled = true;
    
    // Tracking coordinates (stored in inches, converted to meters for calculations)
    private double targetX = 0.0;
    private double targetY = 0.0;
    private boolean trackingPoint = false;
    
    // Origin state (true = blue origin, false = red origin)
    private boolean blueOrigin = TurretConstants.BLUE_ORIGIN;
    
    // Singleton instance references
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private SwervePoseEstimator poseEstimator = SwervePoseEstimator.getInstance();
    
    /** Creates a new Turret subsystem. */
    private Turret() {
        // Initialize the Spark MAX motor using WolfSparkMax wrapper
        turretMotor = new WolfSparkMax(
            TurretConstants.MOTOR_ID,
            SparkMax.MotorType.kBrushless,
            SparkBaseConfig.IdleMode.kBrake,
            40, // Current limit (amps)
            TurretConstants.MOTOR_INVERTED
        );
        
        // Initialize PID controller
        pidController = new PIDController(
            TurretConstants.KP,
            TurretConstants.KI,
            TurretConstants.KD
        );
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(TurretConstants.ANGLE_TOLERANCE);
        
        // Reset encoder to zero position
        resetEncoder();
        
        // Initialize with target coordinates from constants (in inches)
        targetX = TurretConstants.TARGET_X;
        targetY = TurretConstants.TARGET_Y;
        trackingPoint = true;
        
        // Calculate initial angle to target
        updateTargetAngleFromCoordinates(targetX, targetY);
        
        // Initialize SmartDashboard values
        SmartDashboard.putNumber("Turret/TargetAngle", targetAngle);
        SmartDashboard.putNumber("Turret/TargetX", targetX);
        SmartDashboard.putNumber("Turret/TargetY", targetY);
        SmartDashboard.putBoolean("Turret/TrackingPoint", trackingPoint);
        SmartDashboard.putBoolean("Turret/Enabled", true);
        SmartDashboard.putBoolean("Turret/BlueOrigin", blueOrigin);
    }
    
    @Override
    public void periodic() {
        // Update current angle from encoder
        motorPosition = turretMotor.getEncoder().getPosition();
        currentAngle = getAngleFromMotorPosition(motorPosition);
        
        // Get target coordinates from SmartDashboard if tracking point
        if (trackingPoint) {
            targetX = SmartDashboard.getNumber("Turret/TargetX", targetX);
            targetY = SmartDashboard.getNumber("Turret/TargetY", targetY);
            
            // Check if origin was toggled
            boolean newBlueOrigin = SmartDashboard.getBoolean("Turret/BlueOrigin", blueOrigin);
            if (newBlueOrigin != blueOrigin) {
                blueOrigin = newBlueOrigin;
                // Convert target coordinates to new origin
                convertTargetToNewOrigin();
            }
            
            updateTargetAngleFromCoordinates(targetX, targetY);
        } else {
            targetAngle = SmartDashboard.getNumber("Turret/TargetAngle", targetAngle);
        }
        
        // Check if turret is enabled
        isEnabled = SmartDashboard.getBoolean("Turret/Enabled", isEnabled);
        
        // Update motor output if enabled
        if (isEnabled) {
            updateMotorOutput();
        } else {
            turretMotor.set(0);
        }
        
        // Update SmartDashboard with current values
        SmartDashboard.putNumber("Turret/CurrentAngle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("Turret/CurrentAngleRad", currentAngle);
        SmartDashboard.putNumber("Turret/MotorPosition", motorPosition);
        SmartDashboard.putNumber("Turret/MotorVoltage", turretMotor.getAppliedOutput());
        SmartDashboard.putBoolean("Turret/AtTarget", atTarget());
    }
    
    /**
     * Converts target coordinates when origin is toggled.
     * If switching to blue origin: target becomes FIELD_LENGTH - redTargetX
     */
    private void convertTargetToNewOrigin() {
        if (blueOrigin) {
            // Converting red to blue: X_blue = FIELD_LENGTH - X_red
            targetX = TurretConstants.FIELD_LENGTH_X - targetX;
        } else {
            // Converting blue to red: X_red = FIELD_LENGTH - X_blue
            targetX = TurretConstants.FIELD_LENGTH_X - targetX;
        }
        SmartDashboard.putNumber("Turret/TargetX", targetX);
        SmartDashboard.putNumber("Turret/TargetY", targetY);
    }
    
    /**
     * Toggles between red and blue origin.
     * Call this from a button press.
     */
    public void toggleOrigin() {
        blueOrigin = !blueOrigin;
        convertTargetToNewOrigin();
        SmartDashboard.putBoolean("Turret/BlueOrigin", blueOrigin);
    }
    
    /**
     * Checks if using blue origin.
     * @return true if using blue alliance origin
     */
    public boolean isBlueOrigin() {
        return blueOrigin;
    }
    
    /**
     * Sets the target angle for the turret to turn to.
     * @param angleRadians Target angle in radians
     */
    public void setTargetAngle(double angleRadians) {
        // Clamp angle to valid range
        targetAngle = Math.max(TurretConstants.MIN_ANGLE, Math.min(TurretConstants.MAX_ANGLE, angleRadians));
        trackingPoint = false;
        SmartDashboard.putNumber("Turret/TargetAngle", targetAngle);
        SmartDashboard.putBoolean("Turret/TrackingPoint", false);
    }
    
    /**
     * Sets the turret to track a specific point on the field.
     * The turret will calculate the angle needed to face that point.
     * @param pointX X coordinate of the target point (inches)
     * @param pointY Y coordinate of the target point (inches)
     */
    public void turnToPoint(double pointX, double pointY) {
        targetX = pointX;
        targetY = pointY;
        trackingPoint = true;
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Turret/TargetX", targetX);
        SmartDashboard.putNumber("Turret/TargetY", targetY);
        SmartDashboard.putBoolean("Turret/TrackingPoint", true);
        
        // Calculate the angle to the target point
        updateTargetAngleFromCoordinates(targetX, targetY);
    }
    
    /**
     * Updates the target angle based on robot pose and target coordinates.
     * Uses SwervePoseEstimator (no cameras).
     */
    private void updateTargetAngleFromCoordinates(double pointX, double pointY) {
        // Get current robot pose from SwervePoseEstimator (no cameras)
        Pose2d robotPose = poseEstimator.getPose();
        Translation2d robotTranslation = robotPose.getTranslation();
        
        // Convert target coordinates to meters based on current origin
        double targetXMeters = blueOrigin 
            ? Units.inchesToMeters(TurretConstants.FIELD_LENGTH_X - pointX)
            : Units.inchesToMeters(pointX);
        double targetYMeters = Units.inchesToMeters(pointY);
        
        // Convert robot position to match coordinate system
        double robotXMeters = blueOrigin
            ? Units.inchesToMeters(TurretConstants.FIELD_LENGTH_X) - robotTranslation.getX()
            : robotTranslation.getX();
        double robotYMeters = robotTranslation.getY();
        
        // Calculate delta to target
        double deltaX = targetXMeters - robotXMeters;
        double deltaY = targetYMeters - robotYMeters;
        
        // Calculate angle to target (field-relative)
        double angleToTarget = Math.atan2(deltaY, deltaX);
        
        // Adjust for robot's current rotation to get turret angle relative to robot
        double robotRotation = robotPose.getRotation().getRadians();
        double turretAngle = angleToTarget - robotRotation;
        
        // Normalize to [-PI, PI]
        turretAngle = Math.IEEEremainder(turretAngle, 2 * Math.PI);
        
        setTargetAngle(turretAngle);
    }
    
    /**
     * Updates the motor output based on PID controller.
     */
    private void updateMotorOutput() {
        double pidOutput = pidController.calculate(currentAngle, targetAngle);
        
        // Apply voltage-based output (PID output is already in output units)
        turretMotor.set(pidOutput);
    }
    
    /**
     * Checks if the turret is at the target angle.
     * @return true if at target within tolerance
     */
    public boolean atTarget() {
        return pidController.atSetpoint();
    }
    
    /**
     * Resets the turret encoder to zero.
     */
    public void resetEncoder() {
        turretMotor.getEncoder().setPosition(0);
        currentAngle = 0.0;
    }
    
    /**
     * Sets the turret to a specific position (for calibration).
     * @param angleRadians The angle in radians to set as zero
     */
    public void setZeroPosition(double angleRadians) {
        double motorPos = getMotorPositionFromAngle(angleRadians);
        turretMotor.getEncoder().setPosition(motorPos);
    }
    
    /**
     * Gets the current turret angle.
     * @return Current angle in radians
     */
    public double getAngle() {
        return currentAngle;
    }
    
    /**
     * Gets the current turret angle in degrees.
     * @return Current angle in degrees
     */
    public double getAngleDegrees() {
        return Math.toDegrees(currentAngle);
    }
    
    /**
     * Gets the target turret angle.
     * @return Target angle in radians
     */
    public double getTargetAngle() {
        return targetAngle;
    }
    
    /**
     * Checks if the turret is currently tracking a point.
     * @return true if tracking a coordinate point
     */
    public boolean isTrackingPoint() {
        return trackingPoint;
    }
    
    /**
     * Stops tracking the point and holds current position.
     */
    public void stopTracking() {
        trackingPoint = false;
        targetAngle = currentAngle;
        SmartDashboard.putBoolean("Turret/TrackingPoint", false);
    }
    
    /**
     * Disables the turret motor.
     */
    public void disable() {
        isEnabled = false;
        turretMotor.set(0);
        SmartDashboard.putBoolean("Turret/Enabled", false);
    }
    
    /**
     * Enables the turret motor.
     */
    public void enable() {
        isEnabled = true;
        SmartDashboard.putBoolean("Turret/Enabled", true);
    }
    
    /**
     * Stops the turret motor.
     */
    public void stop() {
        turretMotor.set(0);
    }
    
    /**
     * Gets the target X coordinate.
     * @return Target X in inches
     */
    public double getTargetX() {
        return targetX;
    }
    
    /**
     * Gets the target Y coordinate.
     * @return Target Y in inches
     */
    public double getTargetY() {
        return targetY;
    }
    
    /**
     * Sets the target X coordinate (in inches).
     * @param x Target X coordinate
     */
    public void setTargetX(double x) {
        targetX = x;
        SmartDashboard.putNumber("Turret/TargetX", targetX);
        if (trackingPoint) {
            updateTargetAngleFromCoordinates(targetX, targetY);
        }
    }
    
    /**
     * Sets the target Y coordinate (in inches).
     * @param y Target Y coordinate
     */
    public void setTargetY(double y) {
        targetY = y;
        SmartDashboard.putNumber("Turret/TargetY", targetY);
        if (trackingPoint) {
            updateTargetAngleFromCoordinates(targetX, targetY);
        }
    }
    
    /**
     * Converts motor position (rotations) to angle (radians).
     * @param motorPosition Motor encoder position in rotations
     * @return Angle in radians
     */
    private double getAngleFromMotorPosition(double motorPosition) {
        return (motorPosition * TurretConstants.ENCODER_PCONVERSION) + TurretConstants.OFFSET;
    }
    
    /**
     * Converts angle (radians) to motor position (rotations).
     * @param angleRadians Angle in radians
     * @return Motor encoder position in rotations
     */
    private double getMotorPositionFromAngle(double angleRadians) {
        return (angleRadians - TurretConstants.OFFSET) / TurretConstants.ENCODER_PCONVERSION;
    }
    
    /**
     * Gets the motor encoder position.
     * @return Motor position in rotations
     */
    public double getMotorPosition() {
        return motorPosition;
    }
}

