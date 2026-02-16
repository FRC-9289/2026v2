package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command
{
    private Intake intake;
    
    public IntakeCommand(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.pullIn();
    }


    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
