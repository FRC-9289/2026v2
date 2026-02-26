package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretMath {

  public static double getAngleToHub(Pose2d robotPose) {
    double dx = TurretConstants.HUB_X - robotPose.getX();
    double dy = TurretConstants.HUB_Y - robotPose.getY();

    double target = Math.atan2(dy, dx);

    return target;
  }
}