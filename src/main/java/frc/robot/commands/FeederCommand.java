package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeederCommand extends Command {
    private final Feeder feeder = Feeder.getInstance();

    public FeederCommand() {
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.start();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
