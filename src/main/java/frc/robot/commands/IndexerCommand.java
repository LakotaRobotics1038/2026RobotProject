package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;

public class IndexerCommand extends Command {
    private final Indexer indexer = Indexer.getInstance();
    private final IndexerDirection direction;

    public IndexerCommand(IndexerDirection direction) {
        this.direction = direction;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        if (direction == IndexerDirection.INTAKE) {
            indexer.intake();
        } else {
            indexer.dispose();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    public enum IndexerDirection {
        INTAKE,
        DISPOSE
    }
}
