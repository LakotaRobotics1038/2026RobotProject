package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;

public class HubAlignCommand extends Command {
    private static final double P = 0.0;
    private static final double I = 0.0;
    private static final double D = 0.0;

    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final PIDController rotationController;

    public HubAlignCommand(DoubleSupplier xSpeed,
                           DoubleSupplier ySpeed) {
        this.xSpeedSupplier = xSpeed;
        this.ySpeedSupplier = ySpeed;

        this.rotationController = new PIDController(P, I, D);

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveTrain.getState().Pose;

        double nearShooterTargetAngle = getModuleTargetHeading(robotPose,
                ShooterConstants.NEAR_SHOOTER_MODULE_CONSTANTS.translation());
        double farShooterTargetAngle = getModuleTargetHeading(robotPose,
                ShooterConstants.FAR_SHOOTER_MODULE_CONSTANTS.translation());

        double targetAngleRad = Math.atan2(
                Math.sin(nearShooterTargetAngle) + Math.sin(farShooterTargetAngle),
                Math.cos(nearShooterTargetAngle) + Math.cos(farShooterTargetAngle));
        // Shooter modules fire sideways relatively to the robot's forward orientation.
        double shooterAlignedTargetAngleRad = MathUtil.angleModulus(
                targetAngleRad + ShooterConstants.SHOOTER_DIRECTION_FROM_FORWARD_RAD);

        double currentRotationRadians = robotPose.getRotation().getRadians();
        double rotationOutputRadPerSec = rotationController.calculate(currentRotationRadians,
                shooterAlignedTargetAngleRad);

        // drive() expects normalized rotation input; clamp keeps PID output in [-1, 1]
        // before DriveTrain rescales by MAX_ANGULAR_RATE.
        double rotation = MathUtil.clamp(rotationOutputRadPerSec / DriveConstants.MAX_ANGULAR_RATE, -1.0, 1.0);

        driveTrain.setControl(driveTrain.drive(xSpeedSupplier.getAsDouble(), -ySpeedSupplier.getAsDouble(),
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
    }

    private static double getModuleTargetHeading(Pose2d robotPose, Translation2d shooterModuleTranslation) {
        Translation2d moduleFieldPosition = robotPose.getTranslation()
                .plus(shooterModuleTranslation.rotateBy(robotPose.getRotation()));
        Translation2d toHubFromModule = FieldConstants.HUB_POSITION.minus(moduleFieldPosition);
        return toHubFromModule.getAngle().getRadians();
    }
}
