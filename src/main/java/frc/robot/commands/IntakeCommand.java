package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final Intake intake = Intake.getInstance();
    private final IntakeDirection direction;

    public IntakeCommand(IntakeDirection direction) {
        this.direction = direction;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (direction == IntakeDirection.INTAKE) {
            intake.forward();
        } else {
            intake.backward();
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
