package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

public class AcquisitionCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Mode mode;

    public AcquisitionCommand(Mode mode) {
        this.mode = mode;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        if (mode == Mode.INTAKE) {
            acquisition.intake();
        } else if (mode == Mode.DISPOSE) {
            acquisition.dispose();
        } else {
            acquisition.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public enum Mode {
        INTAKE,
        DISPOSE,
        STOP
    }
}
