package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionPivotConstants;
import frc.robot.subsystems.AcquisitionPivot;

public class AcquisitionPivotTrenchRetract extends Command {
    private final AcquisitionPivot pivot = AcquisitionPivot.getInstance();

    public AcquisitionPivotTrenchRetract() {
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setAngle(
                Math.min(pivot.getPivotPosition(), AcquisitionPivotConstants.MAX_TRENCH_ANGLE_DEGREES));
    }

    @Override
    public boolean isFinished() {
        return pivot.pivotAtSetpoint();
    }
}
