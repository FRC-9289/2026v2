package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Roller.Arm;

public class RotateArmToSetpoint extends Command{
    private final Arm arm;
    private final double pos;

    public RotateArmToSetpoint(Arm arm, double pos) {
        this.arm = arm;
        this.pos = pos;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.rotateArmToSetpoint(pos);
    }

    @Override
    public void end(boolean interrupted) {
        arm.rotateArmToSetpoint(0.1
        
        );
    }
}
