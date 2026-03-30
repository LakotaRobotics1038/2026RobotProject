package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

public class RunPrototypeAcquisitionCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Mode mode;

    public RunPrototypeAcquisitionCommand(Mode mode) {
        this.mode = mode;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        if (mode == Mode.INTAKE) {
            acquisition.intake();
        } else if (mode == Mode.DISPOSE) {
            acquisition.dispose();
        }
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        acquisition.stop();
    }

    public enum Mode {
        INTAKE,
        DISPOSE
    }
}
