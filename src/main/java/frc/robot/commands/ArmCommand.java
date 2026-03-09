package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Roller.Arm;
import frc.robot.subsystems.Turret.Turret;

public class ArmCommand extends Command{
    private final Arm arm;
    private BooleanSupplier up;
    private BooleanSupplier down;
    private BooleanSupplier left;

    public ArmCommand(Arm arm, BooleanSupplier up, BooleanSupplier down, BooleanSupplier left) {
        this.arm = arm;
        this.up = up;
        this.down = down;
        this.left = left;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (left.getAsBoolean()) {
            arm.rotateArmToSetpoint(1.5);
        }
        else if (up.getAsBoolean()) {
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
