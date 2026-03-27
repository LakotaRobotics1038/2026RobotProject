package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.utils.Direction;

public class IndexerCommand extends Command {
    private final Indexer indexer = Indexer.getInstance();
    private final Direction direction;

    public IndexerCommand(Direction direction) {
        this.direction = direction;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        if (direction == Direction.FORWARD) {
            indexer.start();
        } else {
            indexer.backward();
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
}
