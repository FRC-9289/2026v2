package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;

public class SwerveCommand extends Command{
    private Swerve swerve;
    private Translation2d translation;
    private boolean isOpenLoop;
    private double rotation;
    private boolean fieldRelative;
    public SwerveCommand(Swerve swerve, Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        this.swerve = swerve;
        this.translation = translation;
        this.rotation = rotation;
        this.fieldRelative = fieldRelative;
        this.isOpenLoop = isOpenLoop;
    }

    @Override
    public void execute(){
        swerve.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    @Override
    public void end(boolean interrupted){
        swerve.drive(new Translation2d(0,0), 0, fieldRelative, isOpenLoop);
    }
}

