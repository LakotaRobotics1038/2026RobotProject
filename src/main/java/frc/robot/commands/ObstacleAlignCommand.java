package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ObstacleAlignCommand extends Command {
    private static final double QUARTER_TURN = Math.PI / 2;
    private static final double EIGHTH_TURN = Math.PI / 4;

    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final DoubleSupplier forwardSpeedSupplier;
    private final DoubleSupplier sidewaysSpeedSupplier;

    public ObstacleAlignCommand(DoubleSupplier forwardSpeedSupplier, DoubleSupplier sidewaysSpeedSupplier) {
        this.forwardSpeedSupplier = forwardSpeedSupplier;
        this.sidewaysSpeedSupplier = sidewaysSpeedSupplier;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveTrain.getState().Pose;
        Translation2d robotPosition = robotPose.getTranslation();
        double currentHeading = robotPose.getRotation().getRadians();

        double targetHeading;
        if (isInTrench(robotPosition)) {
            targetHeading = snapToNearest(currentHeading, QUARTER_TURN, 0);
        } else {
            targetHeading = snapToNearest(currentHeading, QUARTER_TURN, EIGHTH_TURN);
        }

        double rotationOutput = DriveConstants.ROTATION_CONTROLLER.calculate(currentHeading, targetHeading);
        double rotation = MathUtil.clamp(rotationOutput / DriveConstants.MAX_ANGULAR_RATE,
                -DriveConstants.MAX_ROTATION_POWER,
                DriveConstants.MAX_ROTATION_POWER);

        driveTrain.setControl(driveTrain.drive(forwardSpeedSupplier.getAsDouble(),
                -sidewaysSpeedSupplier.getAsDouble(), rotation, true));
    }

    @Override
    public void end(boolean interrupted) {
        DriveConstants.ROTATION_CONTROLLER.reset();
        driveTrain.setControl(driveTrain.drive(forwardSpeedSupplier.getAsDouble(),
                -sidewaysSpeedSupplier.getAsDouble(), 0, true));
    }

    private boolean isInTrench(Translation2d position) {
        return position.getY() < FieldConstants.TRENCH_WIDTH
                || position.getY() > FlippingUtil.fieldSizeY - FieldConstants.TRENCH_WIDTH;
    }

    private double snapToNearest(double headingRadians, double stepRadians, double offsetRadians) {
        double shifted = headingRadians - offsetRadians;
        double snapped = Math.round(shifted / stepRadians) * stepRadians + offsetRadians;
        return MathUtil.angleModulus(snapped);
    }
}