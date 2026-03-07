package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants.ClimbSetpoint;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends Command {
    private final Climb climb = Climb.getInstance();
    private final ClimbSetpoint setpoint;

    public ClimbCommand(ClimbSetpoint setpoint) {
        this.setpoint = setpoint;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return climb.isAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }
}
