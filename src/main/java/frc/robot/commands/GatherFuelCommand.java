package frc.robot.commands;

import frc.robot.controls.TransferCommands.DirectionTransfer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Transfer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

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
        addRequirements(intake, kicker, transfer); // only if supported
    }

    @Override
    public void execute() {
        intake.pullIn();
        kicker.goUp();

        DirectionTransfer dir = direction.get();
        if (dir == DirectionTransfer.FORWARD) {
            transfer.movingForward();
        } else if (dir == DirectionTransfer.BACKWARD) {
            transfer.movingBackward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        kicker.stop();
        transfer.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // run until cancelled
    }
}