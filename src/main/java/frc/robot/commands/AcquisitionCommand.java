package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

public class AcquisitionCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final IntakeDirection direction;

    public AcquisitionCommand(IntakeDirection direction) {
        this.direction = direction;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        if (direction == IntakeDirection.INTAKE) {
            acquisition.intake();
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
        acquisition.stop();
    }

    public enum IntakeDirection {
        INTAKE,
        DISPOSE
    }
}
