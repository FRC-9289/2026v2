package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretMath {

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

  public static double getAngleToHubAdih(Pose2d robotPose, Translation2d target) {
      double requiredTurretHeading = Math.toDegrees(Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX()));
      double offset = (requiredTurretHeading > 0) ? -(requiredTurretHeading - 180) : -(requiredTurretHeading + 180);
      return offset + robotPose.getRotation().getDegrees();
  }
}