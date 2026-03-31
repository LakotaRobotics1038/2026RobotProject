package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ObstacleAlignCommand extends Command {
    private static final double TRENCH_TURN_SNAP_STEP = Math.PI;
    private static final double TRENCH_TURN_SNAP_OFFSET = 0;
    private static final double BUMP_TURN_SNAP_STEP = Math.PI / 2;
    private static final double BUMP_TURN_SNAP_OFFSET = Math.PI / 4;

    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final DoubleSupplier forwardSpeedSupplier;
    private final DoubleSupplier sidewaysSpeedSupplier;
    private final PIDController rotationController = new PIDController(DriveConstants.P, DriveConstants.I,
            DriveConstants.D);

    public ObstacleAlignCommand(DoubleSupplier forwardSpeedSupplier, DoubleSupplier sidewaysSpeedSupplier) {
        this.forwardSpeedSupplier = forwardSpeedSupplier;
        this.sidewaysSpeedSupplier = sidewaysSpeedSupplier;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(DriveConstants.ALIGNMENT_TOLERANCE_RAD);
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveTrain.getState().Pose;
        Translation2d robotPosition = robotPose.getTranslation();
        double currentHeading = robotPose.getRotation().getRadians();

        double targetHeading;
        if (isInTrench(robotPosition)) {
            targetHeading = snapToNearest(currentHeading, TRENCH_TURN_SNAP_STEP, TRENCH_TURN_SNAP_OFFSET);
        } else {
            targetHeading = snapToNearest(currentHeading, BUMP_TURN_SNAP_STEP, BUMP_TURN_SNAP_OFFSET);
        }

        double rotationOutput = rotationController.calculate(currentHeading, targetHeading);
        double rotation = MathUtil.clamp(rotationOutput / DriveConstants.MAX_ANGULAR_RATE,
                -DriveConstants.MAX_ROTATION_POWER,
                DriveConstants.MAX_ROTATION_POWER);

        driveTrain.setControl(driveTrain.drive(forwardSpeedSupplier.getAsDouble(),
                -sidewaysSpeedSupplier.getAsDouble(), rotation, true));
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();
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