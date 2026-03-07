package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.subsystems.Acquisition;

public class AcquisitionPivotCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final AcquisitionConstants.AcquisitionSetpoint setpoint;

    public AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint setpoint) {
        this.setpoint = setpoint;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.setPivot(setpoint);
    }

    @Override
    public boolean isFinished() {
        return acquisition.atSetpoint();
    }
}
