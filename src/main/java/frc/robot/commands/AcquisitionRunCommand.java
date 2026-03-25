package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Kicker;

public class AcquisitionRunCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Mode mode;

    public AcquisitionRunCommand(Mode mode) {
        this.mode = mode;
        addRequirements(acquisition, kicker);
    }

    @Override
    public void initialize() {
        if (mode == Mode.INTAKE) {
            acquisition.acquire();
        } else {
            acquisition.dispose();
        }
        kicker.reverse();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.stopIntake();
        kicker.stop();
    }

    public enum Mode {
        INTAKE,
        DISPOSE
    }
}
