package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;

public class KickerCommand extends Command {
    private final Kicker kicker = Kicker.getInstance();

    public KickerCommand() {
        addRequirements(kicker);
    }

    @Override
    public void initialize() {
        kicker.start();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
    }
}
