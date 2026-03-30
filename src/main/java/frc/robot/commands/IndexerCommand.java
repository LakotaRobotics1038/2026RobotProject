package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;

public class IndexerCommand extends Command {
    private final Indexer indexer = Indexer.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Mode mode;

    public IndexerCommand(Mode mode) {
        this.mode = mode;
        addRequirements(indexer, kicker);
    }

    @Override
    public void initialize() {
        if (mode == Mode.INTAKE) {
            indexer.start();
        } else {
            indexer.stop();
        }
        kicker.reverse();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        kicker.stop();
    }

    public enum Mode {
        INTAKE,
        DISPOSE
    }
}
