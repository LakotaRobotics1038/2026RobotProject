package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Direction;

public class IntakeCommand extends Command {
    private final Intake intake = Intake.getInstance();
    private final Direction direction;

    public IntakeCommand(Direction direction) {
        this.direction = direction;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (direction == Direction.FORWARD) {
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
}
