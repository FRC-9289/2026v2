package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveFF extends SubsystemBase {
    private final Swerve drivetrain;
    private double rampPercent = 0.0;
    private double lastVel = 0.0;
    private double lastTime = 0.0;
    private boolean runningKA = false;

    public enum Mode { FIND_KS, KV_TEST, KA_TEST }

    private Mode mode = Mode.FIND_KS;
    private double fixedPercent = 0.0;

    public SwerveFF(Swerve drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void setMode(Mode mode, double percent) {
        this.mode = mode;
        this.fixedPercent = percent;
        this.rampPercent = 0.0;
        this.lastVel = 0.0;
        this.lastTime = Timer.getFPGATimestamp();
        this.runningKA = (mode == Mode.KA_TEST);
    }

    @Override
    public void periodic() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        double vel = drivetrain.getAverageVelocity(); // meters/sec

        switch (mode) {
            case FIND_KS:
                rampPercent += 0.002;
                drivetrain.driveOpenLoop(rampPercent);
                SmartDashboard.putNumber("kS_percent", rampPercent);
                SmartDashboard.putNumber("velocity", vel);
                if (vel > 0.05) drivetrain.driveOpenLoop(0); // stop after moving
                break;

            case KV_TEST:
                drivetrain.driveOpenLoop(fixedPercent);
                SmartDashboard.putNumber("kV_percent", fixedPercent);
                SmartDashboard.putNumber("velocity", vel);
                break;

            case KA_TEST:
                if (runningKA) {
                    double accel = (vel - lastVel) / dt;
                    SmartDashboard.putNumber("velocity", vel);
                    SmartDashboard.putNumber("acceleration", accel);
                    lastVel = vel;
                    lastTime = Timer.getFPGATimestamp();
                }
                break;
        }
        lastTime = Timer.getFPGATimestamp();
    }

    // Utility to get average drive velocity from Swerve modules
    public double getAverageVelocity() {
        return drivetrain.getAverageVelocity();
    }
}