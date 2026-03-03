package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

public class AcquisitionDisposeCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();

    public AcquisitionDisposeCommand() {
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.dispose();
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.stopIntake();
    }
}
