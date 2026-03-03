package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

public class AcquisitionIntakeCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();

    public AcquisitionIntakeCommand() {
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.acquire();
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.stopIntake();
    }
}
