
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SwervePoseEstimator - A camera-free pose estimator using swerve drive odometry.
 * 
 * This class provides robot pose estimation based solely on:
 * - Swerve module wheel positions (drive encoders)
 * - Gyro heading (Pigeon2)
 * 
 * No cameras or vision system required.
 */
public class SwervePoseEstimator extends SubsystemBase {
    // Singleton instance
    private static final SwervePoseEstimator instance = new SwervePoseEstimator();
    
    public static SwervePoseEstimator getInstance() {
        return instance;
    }
    
    // Reference to drivetrain for module positions and gyro
    private Drivetrain drivetrain = Drivetrain.getInstance();
    
    // Current pose (updated each periodic)
    private Pose2d currentPose = new Pose2d();
    
    // Previous module positions for distance calculation
    private SwerveModulePosition[] previousPositions;
    
    // Whether we've initialized the previous positions
    private boolean initialized = false;
    
    /** Creates a new SwervePoseEstimator. */
    private SwervePoseEstimator() {
        // Initialize previous positions to current
        previousPositions = drivetrain.getModulePositions();
        initialized = true;
    }
    
    @Override
    public void periodic() {
        // Get current module positions and gyro heading
        SwerveModulePosition[] currentPositions = drivetrain.getModulePositions();
        Rotation2d gyroHeading = drivetrain.getHeadingRotation2d();
        
        // Calculate distance moved since last update
        double dx = 0.0;
        double dy = 0.0;
        
        if (initialized) {
            // Average the distance from all modules (for swerve, they're typically close)
            for (int i = 0; i < 4; i++) {
                double deltaDistance = currentPositions[i].distanceMeters - previousPositions[i].distanceMeters;
                // Use the module's current angle to decompose into x/y
                dx += deltaDistance * Math.cos(currentPositions[i].angle.getRadians());
                dy += deltaDistance * Math.sin(currentPositions[i].angle.getRadians());
            }
            dx /= 4.0;
            dy /= 4.0;
        }
        
        // Update previous positions
        previousPositions = currentPositions;
        initialized = true;
        
        // Update current pose
        double newX = currentPose.getX() + dx;
        double newY = currentPose.getY() + dy;
        Rotation2d newRotation = gyroHeading;
        
        currentPose = new Pose2d(newX, newY, newRotation);
        
        // Update SmartDashboard
        SmartDashboard.putNumber("SwervePose/X", currentPose.getX());
        SmartDashboard.putNumber("SwervePose/Y", currentPose.getY());
        SmartDashboard.putNumber("SwervePose/Rotation", currentPose.getRotation().getDegrees());
    }
    
    /**
     * Gets the current estimated pose of the robot.
     * @return Current pose in meters
     */
    public Pose2d getPose() {
        return currentPose;
    }
    
    /**
     * Gets the current robot X position.
     * @return X position in meters
     */
    public double getX() {
        return currentPose.getX();
    }
    
    /**
     * Gets the current robot Y position.
     * @return Y position in meters
     */
    public double getY() {
        return currentPose.getY();
    }
    
    /**
     * Gets the current robot rotation.
     * @return Robot rotation
     */
    public Rotation2d getRotation() {
        return currentPose.getRotation();
    }
    
    /**
     * Resets the pose estimator to a specific position.
     * @param pose New pose to set
     */
    public void resetPose(Pose2d pose) {
        currentPose = pose;
        initialized = false;
    }
    
    /**
     * Resets the pose estimator to origin (0, 0).
     */
    public void resetToOrigin() {
        currentPose = new Pose2d();
        initialized = false;
    }
}

