package frc.robot.utils;

import frc.robot.subsystems.Shooter.ShooterConstants;

public class ShooterMath {
    public static double calculateAngularVelocityFromDistanceToHub(double distanceMeters, double param) {
        double g = 9.81;
        double theta = ShooterConstants.SHOOTER_ANGLE_RAD;
        double deltaH = ShooterConstants.CHANGE_IN_HEIGHT;

        double numerator = g * Math.pow(distanceMeters, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) *
            (distanceMeters * Math.tan(theta) - deltaH);

        double escapeVelocity = Math.sqrt(numerator / denominator);
        double angularVelocity = escapeVelocity / ShooterConstants.ROTATOR_RADIUS;
        return angularVelocity*param;
    }
}
