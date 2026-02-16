package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer.Transfer;

public class TransferCommand extends Command{
    private Transfer transfer;
    private String direction;
        
    
    public TransferCommand(Transfer transfer, String direction){
        this.transfer = transfer;
        this.direction = direction;
        addRequirements(transfer);
    }

    @Override
    public void execute(){
        if(direction.equals("forward")) transfer.movingForward();
        else if(direction.equals("backward")) transfer.movingBackward();
    }


    @Override
    public void end(boolean interrupted){
        transfer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
