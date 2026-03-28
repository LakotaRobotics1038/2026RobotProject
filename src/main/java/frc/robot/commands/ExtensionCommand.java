package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Extension;
import frc.robot.utils.Direction;

public class ExtensionCommand extends Command {
    private final Extension extension = Extension.getInstance();
    private final Direction direction;

    public ExtensionCommand(Direction direction) {
        this.direction = direction;
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        if (direction == Direction.FORWARD) {
            extension.forward();
        } else {
            extension.backward();
        }
    }

    @Override
    public boolean isFinished() {
        if (direction == Direction.FORWARD) {
            return extension.getForwardLimitSwitchPressed();
        } else {
            return extension.getReverseLimitSwitchPressed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        extension.stop();
    }
}
