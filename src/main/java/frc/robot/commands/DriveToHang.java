package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Hang.Hang;

public class DriveToHang extends SequentialCommandGroup {

    public DriveToHang(Swerve swerve, Hang hang) {

        Pose2d hangPose = new Pose2d(
            5.3,
            2.1,
            new edu.wpi.first.math.geometry.Rotation2d(Math.PI)
        );

        PIDController xController = new PIDController(2.5, 0, 0);
        PIDController yController = new PIDController(2.5, 0, 0);
        PIDController rotController = new PIDController(3.0, 0, 0);

        Command driveToPose = new Command() {

            {
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
                        true,
                        false
                );
            }

            @Override
            public boolean isFinished() {
                return swerve.getPose().getTranslation()
                        .getDistance(hangPose.getTranslation()) < 0.1;
            }

            @Override
            public void end(boolean interrupted) {
                swerve.drive(new Translation2d(0,0),0,true,false);
            }
        };

        addCommands(

            /* Step 1: Drive to hang location */
            driveToPose,

            /* Step 2: Run hang */
            new HangCommand(hang, () -> true)

        );
    }
}