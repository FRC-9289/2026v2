package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker.Kicker;

public class KickerCommand extends Command
{
    private Kicker kicker;
    
    public KickerCommand(Kicker kicker)
    {
        this.kicker = kicker;
        addRequirements(kicker);
    }

    @Override
    public void execute(){
        kicker.goUp();
    }


    @Override
    public void end(boolean interrupted){
        kicker.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
