package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.HopperExtension;

public class HopperExtensionCommand extends Command {
    private final HopperExtension extension = HopperExtension.getInstance();
    private final ExtensionDirection direction;
    private final Timer timer = new Timer();

    public HopperExtensionCommand(ExtensionDirection direction) {
        this.direction = direction;
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        if (direction == ExtensionDirection.IN) {
            extension.in();
        } else {
            timer.restart();
            extension.out();
        }
    }

    @Override
    public boolean isFinished() {
        if (direction == ExtensionDirection.OUT) {
            return timer.hasElapsed(Dashboard.HOPPER_OUT_SECONDS.get());
        } else {
            return extension.getReverseLimitSwitchPressed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        extension.stop();
        timer.stop();
    }

    public enum ExtensionDirection {
        IN,
        OUT
    }
}
