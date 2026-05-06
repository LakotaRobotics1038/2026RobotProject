package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HopperExtensionConstants;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.HopperExtension;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;

public class ShootCommand extends Command {
    private static final double IN_SLOW_INTERPOLATE_SECONDS = 0.5;

    private final Kicker kicker = Kicker.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Acquisition acquisition = Acquisition.getInstance();
    private final HopperExtension extension = HopperExtension.getInstance();
    private final Feeder feeder = Feeder.getInstance();
    private final Timer timer = new Timer();

    public ShootCommand() {
        addRequirements(kicker, indexer, acquisition, extension, feeder);
    }

    @Override
    public void initialize() {
        timer.restart();
        kicker.start();
        indexer.intake();
        feeder.start();
        acquisition.intake();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.25)) {
            acquisition.stop();
        }
        if (!timer.hasElapsed(IN_SLOW_INTERPOLATE_SECONDS)) {
            double elapsed = timer.get();
            double interpolatedValue = MathUtil.interpolate(
                    HopperExtensionConstants.IN_DUTY_CYCLE_WHILE_SHOOTING_MIN,
                    HopperExtensionConstants.IN_DUTY_CYCLE_WHILE_SHOOTING_MAX,
                    elapsed / IN_SLOW_INTERPOLATE_SECONDS);
            extension.setSpeed(interpolatedValue);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
        indexer.stop();
        acquisition.stop();
        extension.stop();
        feeder.stop();
        timer.stop();
    }
}
