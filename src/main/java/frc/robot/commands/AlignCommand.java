package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.SwagLights.OperatorStates;

public class AlignCommand extends Command {
    public static final double HUB_ALIGNMENT_RUMBLE_INTENSITY = 0.8;

    private final DriveTrain driveTrain = DriveTrain.getInstance();

    private final Shooter shooter = Shooter.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();
    private final DoubleSupplier forwardSpeedSupplier;
    private final DoubleSupplier sidewaysSpeedSupplier;
    private final PIDController rotationController = new PIDController(DriveConstants.P, DriveConstants.I,
            DriveConstants.D);
    private Boolean alignedToHub;

    public AlignCommand(DoubleSupplier forwardSpeedSupplier,
            DoubleSupplier sidewaysSpeedSupplier) {
        this.forwardSpeedSupplier = forwardSpeedSupplier;
        this.sidewaysSpeedSupplier = sidewaysSpeedSupplier;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(DriveConstants.ALIGNMENT_TOLERANCE_RAD);

        addRequirements(driveTrain, swagLights);
    }

    public AlignCommand() {
        this(() -> 0, () -> 0);
    }

    @Override
    public void initialize() {
        Dashboard.HUB_ALIGNING.set(true);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveTrain.getState().Pose;

        double targetHeadingRadians = getAlignedTargetHeading(robotPose);
        double currentHeadingRadians = robotPose.getRotation().getRadians();

        double rotationOutput = rotationController.calculate(currentHeadingRadians,
                targetHeadingRadians);
        updateAlignmentState(rotationController.atSetpoint());

        double rotation = MathUtil.clamp(rotationOutput / DriveConstants.MAX_ANGULAR_RATE,
                -DriveConstants.MAX_ROTATION_POWER,
                DriveConstants.MAX_ROTATION_POWER);

        driveTrain.setControl(driveTrain.drive(forwardSpeedSupplier.getAsDouble(), -sidewaysSpeedSupplier.getAsDouble(),
                rotation,
                true));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.reset();
        driveTrain.setControl(
                driveTrain.drive(forwardSpeedSupplier.getAsDouble(), -sidewaysSpeedSupplier.getAsDouble(), 0, true));
        updateAlignmentState(false);
        swagLights.setOperatorState(SwagLights.OperatorStates.Default);
        Dashboard.HUB_ALIGNING.set(false);
    }

    private double getAlignedTargetHeading(Pose2d robotPose) {
        Translation2d toTarget = FieldConstants.targetPosition(robotPose.getTranslation()).minus(robotPose.getTranslation());
        // Point the back of the robot at the target (180 degrees from front)
        return MathUtil.angleModulus(toTarget.getAngle().getRadians() + Math.PI);
    }

    private void updateAlignmentState(boolean isAligned) {
        if (alignedToHub == null || alignedToHub != isAligned) {
            alignedToHub = isAligned;
            if (alignedToHub) {
                swagLights.setOperatorState(OperatorStates.Aligned);
            } else {
                swagLights.setOperatorState(OperatorStates.Aligning);
            }
            Dashboard.HUB_ALIGNED.set(isAligned);
        }
    }
}
