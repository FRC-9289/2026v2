package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Shooter.Shooter;

public class SetInitialPose extends InstantCommand{
    private double x;
    private double y;
    private double theta;
    private BooleanSupplier condition;
    private Swerve swerve;
    public SetInitialPose(Swerve swerve, double x, double y, double theta, BooleanSupplier condition) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.condition = condition;
        this.swerve = swerve;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(RobotContainer.swerve);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setPose(new Pose2d(
            new Translation2d(x,y),
            new Rotation2d(theta)
        ));
    }

    
}
