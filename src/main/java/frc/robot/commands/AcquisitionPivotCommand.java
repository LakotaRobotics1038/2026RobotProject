package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;
import frc.robot.subsystems.AcquisitionPivot;

public class AcquisitionPivotCommand extends Command {
    private final AcquisitionPivot acquisition = AcquisitionPivot.getInstance();
    private final PivotSetpoint setpoint;

    public AcquisitionPivotCommand(PivotSetpoint setpoint) {
        this.setpoint = setpoint;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.setAngleSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return acquisition.isAtSetpoint();
    }
}
