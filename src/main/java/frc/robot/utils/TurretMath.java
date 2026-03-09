package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretMath {

  public static double getAngleToHub(Pose2d robotPose) {
    double dx = TurretConstants.HUB_X - robotPose.getX();
    double dy = TurretConstants.HUB_Y - robotPose.getY();

    double target = Math.atan2(dy, dx);

    return target;
  }

  public static Rotation2d getDesiredTurretAngle(Pose2d robotPose, Translation2d target) {

      // vector from robot to target
      double dx = target.getX() - robotPose.getX();
      double dy = target.getY() - robotPose.getY();

      // angle in field coordinates
      double targetGlobalAngle = Math.atan2(dy, dx);

      // convert to robot-relative turret angle
      double desiredTurretAngle = targetGlobalAngle - robotPose.getRotation().getRadians();

      // normalize between -pi and pi
      desiredTurretAngle = MathUtil.angleModulus(desiredTurretAngle);

      return new Rotation2d(desiredTurretAngle);
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