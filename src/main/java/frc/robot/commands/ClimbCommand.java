package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends Command {
    private final Climb climb = Climb.getInstance();
    private final Direction direction;

    public ClimbCommand(Direction direction) {
        this.direction = direction;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        if (direction == Direction.UP) {
            climb.climbUp();
        } else {
            climb.climbDown();
        }
    }

    @Override
    public boolean isFinished() {
        return climb.isAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }

    public enum Direction {
        UP, DOWN
    }
}
