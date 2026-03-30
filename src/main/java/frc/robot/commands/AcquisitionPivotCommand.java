package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants.PivotSetpoint;
import frc.robot.subsystems.Pivot;

public class AcquisitionPivotCommand extends Command {
    private final Pivot acquisition = Pivot.getInstance();
    private final PivotSetpoint setpoint;

    public AcquisitionPivotCommand(PivotSetpoint setpoint) {
        this.setpoint = setpoint;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.setPivot(setpoint);
    }

    @Override
    public boolean isFinished() {
        return acquisition.pivotAtSetpoint();
    }
}
