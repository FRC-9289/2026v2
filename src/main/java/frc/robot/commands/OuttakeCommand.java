package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Outtake.Outtake;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;

public class OuttakeCommand extends Command {
    private Outtake module;
    private Swerve swerve;
    private BooleanSupplier activeSup;
    private static boolean active;

    private static Translation2d blueHub = new Translation2d(4.49341875, 4.03463125);

    public OuttakeCommand(Outtake module, Swerve swerve, BooleanSupplier x) {
        this.module = module;
        this.swerve = swerve;

        this.activeSup = x;

        addRequirements(module);
    }

    @Override
    public void execute() {
        if (this.activeSup.getAsBoolean()) {
            this.active = !this.active;
        }

        if (active) {
            this.module.
            pull(1);
            this.module.carry(1);
            // this.module.turret(30 * (Math.atan2(blueHub.getY() - this.swerve.getPose().getY(), blueHub.getX() - this.swerve.getPose().getX()) * 180 / Math.PI - this.swerve.getGyroYaw().getDegrees()));
            // this.module.wheel(blueHub.getDistance(this.swerve.getPose().getTranslation()) / blueHub.getDistance(new Translation2d(0, 0)));

            /*Table of true scores:
             * 115 in, .92
            */
            this.module.launcher(blueHub.getDistance(this.swerve.getPose().getTranslation()) * 0.314960629921259); //.3 is slope
        } else {
            this.module.carry(0);
            // this.module.turret(0);
            this.module.launcher(0);
        }
    }
}
//Wolfram121