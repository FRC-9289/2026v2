package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Hang.Hang;
import frc.lib.util.HangUtil;

public class DriveToHang extends SequentialCommandGroup {

    public DriveToHang(Swerve swerve, Hang hang) {

        PIDController xController = new PIDController(2.5,0,0);
        PIDController yController = new PIDController(2.5,0,0);
        PIDController rotController = new PIDController(3,0,0);

        addCommands(

            new edu.wpi.first.wpilibj2.command.Command() {

                {
                    addRequirements(swerve);
                }

                @Override
                public void execute() {

                    var targetPose = HangUtil.getRightHangPose();
                    var currentPose = swerve.getPose();

                    double xSpeed = xController.calculate(
                        currentPose.getX(),
                        targetPose.getX()
                    );

                    double ySpeed = yController.calculate(
                        currentPose.getY(),
                        targetPose.getY()
                    );

                    double rotSpeed = rotController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians()
                    );

                    swerve.drive(
                        new Translation2d(xSpeed, ySpeed),
                        rotSpeed,
                        true,
                        false
                    );
                }

                @Override
                public boolean isFinished() {

                    return swerve.getPose().getTranslation()
                        .getDistance(HangUtil.getRightHangPose().getTranslation()) < 0.15;
                }

                @Override
                public void end(boolean interrupted) {

                    swerve.drive(new Translation2d(),0,true,false);
                }

            },

            // Run hang after driving
            new HangCommand(hang, () -> true)

        );
    }
}