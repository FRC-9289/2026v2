package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HangUtil {

    // Field length from diagram
    private static final double FIELD_LENGTH = 16.541; // meters

    // Right hang location (meters)
    private static final double HANG_X_BLUE = 3.64;
    private static final double HANG_Y = 1.19;

    public static Pose2d getRightHangPose() {

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red) {

            return new Pose2d(
                FIELD_LENGTH - HANG_X_BLUE,
                HANG_Y,
                Rotation2d.fromDegrees(0) // face field edge
            );

        } else {

            return new Pose2d(
                HANG_X_BLUE,
                HANG_Y,
                Rotation2d.fromDegrees(180) // face field edge
            );
        }
    }
}