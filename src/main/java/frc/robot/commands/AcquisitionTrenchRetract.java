package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class AcquisitionTrenchRetract extends Command {
    private final Pivot acquisition = Pivot.getInstance();

    public AcquisitionTrenchRetract() {
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.setPivotDegrees(
                Math.min(acquisition.getPivotPosition(), PivotConstants.MAX_TRENCH_ANGLE_DEGREES));
    }

    @Override
    public boolean isFinished() {
        return acquisition.pivotAtSetpoint();
    }
}
