package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

public class AcquisitionCommand extends Command {
    private final Acquisition intake = Acquisition.getInstance();
    private final IntakeDirection direction;

    public AcquisitionCommand(IntakeDirection direction) {
        this.direction = direction;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (direction == IntakeDirection.INTAKE) {
            intake.intake();
        } else {
            intake.dispose();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    public enum IntakeDirection {
        INTAKE,
        DISPOSE
    }
}
