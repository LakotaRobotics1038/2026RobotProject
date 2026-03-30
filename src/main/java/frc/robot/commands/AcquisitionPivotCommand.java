package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;
import frc.robot.subsystems.AcquisitionPivot;

public class AcquisitionPivotCommand extends Command {
    private final AcquisitionPivot pivot = AcquisitionPivot.getInstance();
    private final PivotSetpoint setpoint;

    public AcquisitionPivotCommand(PivotSetpoint setpoint) {
        this.setpoint = setpoint;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setAngleSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return pivot.isAtSetpoint();
    }
}
