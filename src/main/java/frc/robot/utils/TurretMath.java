package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretMath {

  public static double getAngleToHub(Pose2d robotPose) {
    double dx = TurretConstants.HUB_X - robotPose.getX();
    double dy = TurretConstants.HUB_Y - robotPose.getY();

    double target = Math.atan2(dy, dx);
    double robotHeading = robotPose.getRotation().getRadians();

    return MathUtil.angleModulus(target - robotHeading);
  }

  public static double getAngleToHubAdih(Pose2d robotPose){
    // double requiredTurretHeading = -robotPose.getRotation().getRadians() + (Math.PI/2 - Math.atan2(-(robotPose.getY()+0.45), 
    // -(robotPose.getX()+0.45)));
    double requiredTurretHeading = Math.toDegrees(Math.atan2((robotPose.getY()),(robotPose.getX())));
    if (requiredTurretHeading > 0){
      return -(requiredTurretHeading-180) +robotPose.getRotation().getDegrees();
    }else{
      return -(requiredTurretHeading+180)+robotPose.getRotation().getDegrees();
    }
    
  }
}