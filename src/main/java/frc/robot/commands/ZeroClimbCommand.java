package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ZeroClimbCommand extends Command {
    private final Climb climb = Climb.getInstance();

    public ZeroClimbCommand() {
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.zero();
    }

    @Override
    public boolean isFinished() {
        return climb.limitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }
}