package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class HubAlign extends Command {
    private static final double P = 0.0;
    private static final double I = 0.0;
    private static final double D = 0.0;

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
        Translation2d currentTranslation = driveTrain.getState().Pose.getTranslation();
        double targetAngleRad = Math.atan2(
                FieldConstants.HUB_POSITION.getY() - currentTranslation.getY(),
                FieldConstants.HUB_POSITION.getX() - currentTranslation.getX());

        double currentRotationRadians = driveTrain.getState().Pose.getRotation().getRadians();
        double rotationOutputRadPerSec = rotationController.calculate(currentRotationRadians, targetAngleRad);

        driveTrain.setControl(driveTrain.drive(xSpeedSupplier.getAsDouble(), ySpeedSupplier.getAsDouble(),
                MathUtil.clamp(rotationOutputRadPerSec / DriveConstants.MAX_ANGULAR_RATE, -1.0, 1.0),
                true));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
