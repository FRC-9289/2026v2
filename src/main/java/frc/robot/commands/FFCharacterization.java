package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;

public class FFCharacterization extends Command {

    public enum Mode { FIND_KS, KV_TEST, KA_TEST }

    private final Swerve drivetrain;
    private final Mode mode;
    private final double fixedPercent;

    private double rampPercent = 0.0;
    private double lastVel = 0.0;
    private double lastTime = 0.0;

    public FFCharacterization(Swerve drivetrain, Mode mode, double fixedPercent) {
        this.drivetrain = drivetrain;
        this.mode = mode;
        this.fixedPercent = fixedPercent;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        rampPercent = 0.0;
        lastVel = 0.0;
        if (mode == Mode.KA_TEST) {
            drivetrain.driveOpenLoop(0.5); // step input for acceleration
        }
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        double vel = drivetrain.getAverageVelocity();

        switch (mode) {
            case FIND_KS:
                rampPercent += 0.002;
                drivetrain.driveOpenLoop(rampPercent);
                SmartDashboard.putNumber("kS_percent", rampPercent);
                SmartDashboard.putNumber("velocity", vel);
                break;

            case KV_TEST:
                drivetrain.driveOpenLoop(fixedPercent);
                SmartDashboard.putNumber("kV_percent", fixedPercent);
                SmartDashboard.putNumber("velocity", vel);
                break;

            case KA_TEST:
                double accel = (vel - lastVel) / dt;
                SmartDashboard.putNumber("velocity", vel);
                SmartDashboard.putNumber("acceleration", accel);
                lastVel = vel;
                break;
        }
        lastTime = now;
    }

    @Override
    public boolean isFinished() {
        if (mode == Mode.FIND_KS) {
            return drivetrain.getAverageVelocity() > 0.05; // stop when robot starts moving
        }
        return false; // KV and KA tests run until manually canceled
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveOpenLoop(0);
    }
}