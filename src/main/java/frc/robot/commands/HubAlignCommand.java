package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class HubAlignCommand extends Command {
    private static final double P = 0.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double MAX_ROTATION_POWER = 1.0;

    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final PIDController rotationController = new PIDController(P, I, D);

    public HubAlignCommand(DoubleSupplier xSpeed,
                           DoubleSupplier ySpeed) {
        this.xSpeedSupplier = xSpeed;
        this.ySpeedSupplier = ySpeed;

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveTrain.getState().Pose;

        double nearShooterTargetAngle = shooter.getNearShooter().getHubAngle(robotPose);
        double farShooterTargetAngle = shooter.getFarShooter().getHubAngle(robotPose);

        double targetAngleRad = Math.atan2(
                Math.sin(nearShooterTargetAngle) + Math.sin(farShooterTargetAngle),
                Math.cos(nearShooterTargetAngle) + Math.cos(farShooterTargetAngle));
        // Shooter modules fire sideways relatively to the robot's forward orientation.
        double shooterAlignedTargetAngleRad = MathUtil.angleModulus(
                targetAngleRad + ShooterConstants.SHOOTER_DIRECTION_FROM_FORWARD_RAD);

        double currentRotationRadians = robotPose.getRotation().getRadians();
        double rotationOutputRadPerSec = rotationController.calculate(currentRotationRadians,
                shooterAlignedTargetAngleRad);

        double rotation = MathUtil.clamp(rotationOutputRadPerSec / DriveConstants.MAX_ANGULAR_RATE, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);

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
}
