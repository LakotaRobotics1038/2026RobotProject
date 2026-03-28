package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrototypeAcq;

public class RunPrototypeAcquisitionCommand extends Command {
    private final PrototypeAcq prototypeAcq = PrototypeAcq.getInstance();
    private final Mode mode;

    public RunPrototypeAcquisitionCommand(Mode mode) {
        this.mode = mode;
        addRequirements(prototypeAcq);
    }

    @Override
    public void initialize() {
        if (mode == Mode.INTAKE) {
            prototypeAcq.intake();
        } else if (mode == Mode.DISPOSE) {
            prototypeAcq.dispose();
        }
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        prototypeAcq.stop();
    }

    public enum Mode {
        INTAKE,
        DISPOSE
    }
}
