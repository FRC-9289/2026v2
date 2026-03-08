package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;

public class DriveToHang extends Command {

    private final Swerve swerve;

    // Target hang location on field
    private final Pose2d hangPose = new Pose2d(5.3, 2.1, new edu.wpi.first.math.geometry.Rotation2d(Math.PI));

    private final PIDController xController = new PIDController(2.5, 0, 0);
    private final PIDController yController = new PIDController(2.5, 0, 0);
    private final PIDController rotController = new PIDController(3.0, 0, 0);

    public DriveToHang(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {

        Pose2d currentPose = swerve.getPose();

        double xSpeed = xController.calculate(
            currentPose.getX(),
            hangPose.getX()
        );

        double ySpeed = yController.calculate(
            currentPose.getY(),
            hangPose.getY()
        );

        double rotSpeed = rotController.calculate(
            currentPose.getRotation().getRadians(),
            hangPose.getRotation().getRadians()
        );

        Translation2d translation = new Translation2d(xSpeed, ySpeed);

        swerve.drive(
            translation,
            rotSpeed,
            true,   // field relative
            false   // closed loop
        );
    }

    @Override
    public boolean isFinished() {
        return swerve.getPose().getTranslation()
            .getDistance(hangPose.getTranslation()) < 0.1;
    }
}