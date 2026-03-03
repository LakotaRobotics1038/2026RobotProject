package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.subsystems.Acquisition;

public class PivotAcquisitionCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final AcquisitionConstants.AcquisitionSetpoint setpoint;

    public PivotAcquisitionCommand(AcquisitionConstants.AcquisitionSetpoint setpoint) {
        addRequirements(acquisition);
        this.setpoint = setpoint;
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
