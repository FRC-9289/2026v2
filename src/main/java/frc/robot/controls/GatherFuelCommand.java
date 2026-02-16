package frc.robot.controls;

import frc.robot.controls.TransferCommands.DirectionTransfer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Transfer.Transfer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class GatherFuelCommand extends Command {

    private final Intake intake;
    private final Kicker kicker;
    private final Transfer transfer;
    private final Supplier<DirectionTransfer> direction;

    public GatherFuelCommand(
            Intake intake,
            Kicker kicker,
            Transfer transfer,
            Supplier<DirectionTransfer> direction
    ) {
        this.intake = intake;
        this.kicker = kicker;
        this.transfer = transfer;
        this.direction = direction;
        addRequirements(intake, kicker, transfer);
    }

    @Override
    public void execute() {
        intake.pullIn();
        kicker.goUp();

        if (direction.get() == DirectionTransfer.FORWARD) {
            transfer.movingForward();
        } else if (direction.get() == DirectionTransfer.BACKWARD) {
            transfer.movingBackward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        kicker.stop();
        transfer.stop();
    }
}