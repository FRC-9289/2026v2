package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TurretCommands extends Command {
    private final Turret turret;
    private final Drivetrain drivetrain;
    private double targetAngle;
    private double targetX;
    private double targetY;
    private boolean isTrackingPoint;
    
    /** Creates a new TurretCommand to turn to a specific angle. */
    public TurretCommands(double angleRadians) {
        turret = Turret.getInstance();
        drivetrain = Drivetrain.getInstance();
        this.targetAngle = angleRadians;
        this.isTrackingPoint = false;
        addRequirements(turret);
    }
    
    /** Creates a new TurretCommand to turn toward a specific point on the field. */
    public TurretCommands(double x, double y, boolean trackPoint) {
        turret = Turret.getInstance();
        drivetrain = Drivetrain.getInstance();
        this.targetX = x;
        this.targetY = y;
        this.isTrackingPoint = trackPoint;
        addRequirements(turret);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (isTrackingPoint) {
            turret.turnToPoint(targetX, targetY);
        } else {
            turret.setTargetAngle(targetAngle);
        }
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // The turret subsystem handles the actual control in periodic()
        // This command just sets the target once in initialize()
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            turret.stop();
        }
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turret.atTarget();
    }
}

