package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretMath {

  private static Translation2d blueHub = new Translation2d(0.0, 0.0);
  private static final double GEAR_RATIO = 40;

  public static double getAngleToHub(Pose2d robotPose) {
    double dx = TurretConstants.HUB_X - robotPose.getX();
    double dy = TurretConstants.HUB_Y - robotPose.getY();

    double target = Math.atan2(dy, dx);

    return target;
  }

  public static double getDesiredTurretAngle(Pose2d robotPose, Translation2d target) {
    
    double angle = GEAR_RATIO * (Units.radiansToDegrees(Math.atan2(blueHub.getY() - robotPose.getY(), blueHub.getX() - robotPose.getX())) - robotPose.getRotation().getDegrees());
    return angle;
  }
}