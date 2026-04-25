package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperExtension;

public class HopperExtensionCommand extends Command {
    private final HopperExtension extension = HopperExtension.getInstance();
    private final ExtensionDirection direction;

    public HopperExtensionCommand(ExtensionDirection direction) {
        this.direction = direction;
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        if (direction == ExtensionDirection.IN) {
            extension.in();
        } else {
            extension.out();
        }
    }

    @Override
    public boolean isFinished() {
        if (direction == ExtensionDirection.IN) {
            return extension.getReverseLimitSwitchPressed();
        } else {
            return extension.isAtSetpoint();
        }
    }

    @Override
    public void end(boolean interrupted) {
        extension.stop();
    }

    public enum ExtensionDirection {
        IN,
        OUT
    }
}
