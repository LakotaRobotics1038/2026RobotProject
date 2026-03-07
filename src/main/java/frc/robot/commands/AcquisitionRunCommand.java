package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

public class AcquisitionRunCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Mode mode;

    public AcquisitionRunCommand(Mode mode) {
        this.mode = mode;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        if (mode == Mode.INTAKE) {
            acquisition.acquire();
        } else {
            acquisition.dispose();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.stopIntake();
    }

    public enum Mode {
        INTAKE,
        DISPOSE
    }
}
