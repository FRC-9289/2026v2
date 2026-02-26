package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Turret.Turret;

public class ArmCommand extends Command{
    private final Arm arm;
    private final BooleanSupplier up;
    private final BooleanSupplier down;

    public ArmCommand(Arm arm, BooleanSupplier up, BooleanSupplier down) {
        this.arm = arm;
        this.up = up;
        this.down = down;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (up.getAsBoolean()) {
            arm.rotateArm(0.3);
        } 
        else if (down.getAsBoolean()) {
            arm.rotateArm(-0.3);
        } 
        else {
            arm.rotateArm(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.rotateArm(0.0);
    }
}
