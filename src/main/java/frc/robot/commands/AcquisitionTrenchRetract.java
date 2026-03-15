package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.subsystems.Acquisition;

public class AcquisitionTrenchRetract extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();

    public AcquisitionTrenchRetract() {
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.setPivotDegrees(
                Math.min(acquisition.getPivotPosition(), AcquisitionConstants.PIVOT_MAX_TRENCH_ANGLE_DEGREES));
    }

    @Override
    public boolean isFinished() {
        return acquisition.pivotAtSetpoint();
    }
}
