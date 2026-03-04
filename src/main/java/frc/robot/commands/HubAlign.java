package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;

public class HubAlign extends Command {
    private static final double P = 0.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double MIN_DISTANCE_TO_HUB_METERS = 1e-6;

    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final PIDController rotationController;

    public HubAlign(DoubleSupplier xSpeed,
            DoubleSupplier ySpeed) {
        this.xSpeedSupplier = xSpeed;
        this.ySpeedSupplier = ySpeed;

        this.rotationController = new PIDController(P, I, D);

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        Translation2d robotTranslation = driveTrain.getState().Pose.getTranslation();
        Translation2d toHubFromRobotCenter = FieldConstants.HUB_POSITION.minus(robotTranslation);

        double nearShooterTargetAngle = getModuleTargetHeading(toHubFromRobotCenter,
                ShooterConstants.NEAR_SHOOTER_MODULE_CONSTANTS.translation().getY());
        double farShooterTargetAngle = getModuleTargetHeading(toHubFromRobotCenter,
                ShooterConstants.FAR_SHOOTER_MODULE_CONSTANTS.translation().getY());
        double targetAngleRad = Math.atan2(
                Math.sin(nearShooterTargetAngle) + Math.sin(farShooterTargetAngle),
                Math.cos(nearShooterTargetAngle) + Math.cos(farShooterTargetAngle));

        double currentRotationRadians = driveTrain.getState().Pose.getRotation().getRadians();
        double rotationOutputRadPerSec = rotationController.calculate(currentRotationRadians, targetAngleRad);

        driveTrain.setControl(driveTrain.drive(xSpeedSupplier.getAsDouble(), -ySpeedSupplier.getAsDouble(),
                MathUtil.clamp(rotationOutputRadPerSec / DriveConstants.MAX_ANGULAR_RATE, -1.0, 1.0),
                true));
    }

    private static double getModuleTargetHeading(Translation2d toHubFromRobotCenter, double moduleLateralOffset) {
        double baseTargetHeading = Math.atan2(toHubFromRobotCenter.getY(), toHubFromRobotCenter.getX());
        double hubDistance = toHubFromRobotCenter.getNorm();

        if (hubDistance < MIN_DISTANCE_TO_HUB_METERS) {
            return baseTargetHeading;
        }

        double clampedRatio = MathUtil.clamp(moduleLateralOffset / hubDistance, -1.0, 1.0);
        return baseTargetHeading - Math.asin(clampedRatio);
    }
}
