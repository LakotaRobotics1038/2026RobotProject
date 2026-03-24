package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwagLights;

public class AlignCommand extends Command {
    private static final double P = 4.0;
    private static final double I = 0.005;
    private static final double D = 0.01;
    private static final double MAX_ROTATION_POWER = 1.0;
    private static final double ALIGNMENT_TOLERANCE_RAD = Math.toRadians(5.0);
    public static final double HUB_ALIGNMENT_RUMBLE_INTENSITY = 0.8;

    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();
    private final DoubleSupplier forwardSpeedSupplier;
    private final DoubleSupplier sidewaysSpeedSupplier;
    private final BooleanConsumer alignmentStateConsumer;
    private final PIDController rotationController = new PIDController(P, I, D);
    private Boolean alignedToHub;

    public AlignCommand(DoubleSupplier forwardSpeedSupplier,
            DoubleSupplier sidewaysSpeedSupplier,
            BooleanConsumer alignmentStateConsumer) {
        this.forwardSpeedSupplier = forwardSpeedSupplier;
        this.sidewaysSpeedSupplier = sidewaysSpeedSupplier;
        this.alignmentStateConsumer = alignmentStateConsumer;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(ALIGNMENT_TOLERANCE_RAD);

        addRequirements(driveTrain, swagLights);
    }

    public AlignCommand() {
        this(() -> 0, () -> 0, null);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveTrain.getState().Pose;

        double targetHeadingRadians = getAlignedTargetHeading(robotPose);
        double currentHeadingRadians = robotPose.getRotation().getRadians();

        double rotationOutput = rotationController.calculate(currentHeadingRadians, targetHeadingRadians);
        updateAlignmentState(rotationController.atSetpoint());

        double rotation = MathUtil.clamp(rotationOutput / DriveConstants.MAX_ANGULAR_RATE, -MAX_ROTATION_POWER,
                MAX_ROTATION_POWER);

        driveTrain.setControl(driveTrain.drive(forwardSpeedSupplier.getAsDouble(), -sidewaysSpeedSupplier.getAsDouble(),
                rotation,
                true));
    }

    @Override
    public boolean isFinished() {
        if (this.alignmentStateConsumer == null) {
            return rotationController.atSetpoint();
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();
        driveTrain.setControl(
                driveTrain.drive(forwardSpeedSupplier.getAsDouble(), -sidewaysSpeedSupplier.getAsDouble(), 0, true));
        updateAlignmentState(false);
        swagLights.setDefaultState();
    }

    private double getAlignedTargetHeading(Pose2d robotPose) {
        double nearShooterTargetAngle = shooter.getNearShooter().getTargetAngle(robotPose);
        double farShooterTargetAngle = shooter.getFarShooter().getTargetAngle(robotPose);

        // Converts both aim angles into vectors, add them, and turns the result back
        // into an angle. This gives the bisector between the near and far shooter
        // headings without breaking when the angles wrap around -pi and pi.
        double targetAngleRad = Math.atan2(
                Math.sin(nearShooterTargetAngle) + Math.sin(farShooterTargetAngle),
                Math.cos(nearShooterTargetAngle) + Math.cos(farShooterTargetAngle));
        return MathUtil.angleModulus(targetAngleRad + ShooterConstants.SHOOTER_DIRECTION_FROM_FORWARD_RAD);
    }

    private void updateAlignmentState(boolean isAligned) {
        if (alignedToHub == null || alignedToHub != isAligned) {
            alignedToHub = isAligned;
            if (alignedToHub) {
                swagLights.setAlignedState();
            } else {
                swagLights.setAligningState();
            }
            if (alignmentStateConsumer != null) {
                alignmentStateConsumer.accept(alignedToHub);
            }
            dashboard.setHubAligned(isAligned);
        }
    }
}
