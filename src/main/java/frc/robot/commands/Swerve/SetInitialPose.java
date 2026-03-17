package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Turret.Turret;

public class SetInitialPose extends Command{
    private Swerve swerve;
    private Turret turret;

    public SetInitialPose(Swerve swerve, Turret turret){
        this.swerve=swerve;
        this.turret=turret;
        addRequirements(swerve, turret);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("reset: ", "Yes");
        swerve.setInitialPose(new Pose2d(
            new Translation2d(-1, -0.5),
            new Rotation2d(0.0)
        ));
        Turret.enableTracking();
        turret.resetHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
