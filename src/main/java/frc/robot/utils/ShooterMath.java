package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ShooterMath 
{
    // hub coords
    private static final double HUB_X = 0; // need to measure
    private static final double HUB_Y = 0; // need to measure
    private static final double TARGET_HEIGHT = 2.64; // need to measure
    private static final double CAMERA_HEIGHT = 0.6; // need to measure
    private static final double CAMERA_ANGLE = 30.0; // need to measure

    // regression model values
    private static final double A = 0.5; // need to measure
    private static final double B = -2.1; // need to measure
    private static final double C = 15.3; // need to measure
    private static final double D = 20.0; // need to measure
    private static final double E = 2500.0; // need to measure

    // compensation for RPM drop between rapid shots
    private static final double RPM_DROP_COMPENSATION = 50.0; // need to measure

    public static double getOdometryDistance(Pose2d robotPose) 
    {
        Translation2d hubLocation = new Translation2d(HUB_X, HUB_Y);
        return robotPose.getTranslation().getDistance(hubLocation);
    }

    public static double getLimelightDistance() 
    {
        boolean tv = LimelightHelpers.getTV("limelight");
        
        // if we don't see a target, return a negative value as an error flag
        if (!tv) 
        {
            return -1.0;
        }

        double ty = LimelightHelpers.getTY("limelight");
        
        // limelight distance formula
        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        
        // avoid divide by zero if angle is perfectly horizontal
        if (Math.abs(angleToGoalRadians) < 0.001) 
        {
            return -1.0; 
        }
        
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
    }

    public static double getBestDistance(Pose2d robotPose) 
    {
        double llDistance = getLimelightDistance();
        
        // trust limelight if it sees the target, otherwise fall back to swerve odometry
        if (llDistance > 0.0) 
        {
            return llDistance;
        } 
        else 
        {
            return getOdometryDistance(robotPose);
        }
    }

    public static double calculateRPM(double distanceMeters, int consecutiveShots) 
    {
        // quartic regression model: RPM = Ax^4 + Bx^3 + Cx^2 + Dx + E
        double x = distanceMeters;
        double x2 = x * x;
        double x3 = x2 * x;
        double x4 = x3 * x;
        
        double targetRPM = (A * x4) + (B * x3) + (C * x2) + (D * x) + E;
        
        // adjust for voltage/rpm drop if we've fired recently
        // we add a small flat rpm boost per consecutive shot so the next ball doesn't fall short
        double compensation = consecutiveShots * RPM_DROP_COMPENSATION;
        
        // keep it from adjusting too harshly by capping the maximum boost
        compensation = Math.min(compensation, 200.0); // need to measure
        
        return targetRPM + compensation;
    }

    public static double calculateAngularVelocityFromDistanceToHub(double distanceMeters) {
        double numerator = ShooterConstants.G*Math.pow(distanceMeters,2);

        double denominator = 2 * Math.pow(Math.cos(ShooterConstants.THETA_RAD), 2)*(
            distanceMeters*Math.tan(ShooterConstants.THETA_RAD) - ShooterConstants.CHANGE_IN_HEIGHT
        );

        if(denominator<=0){
            return Double.NaN;
        }

        double escapeVelocity = Math.sqrt(numerator/denominator);
        double angularVelocity = escapeVelocity/ShooterConstants.ROTATOR_RADIUS;
        return angularVelocity;
    }
}
