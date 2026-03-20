package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.zip.ZipEntry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.*;

public class ShootCommand extends Command {
    private static final double HOOD_SERVO_MOVE_TIME = 0.5;

    private final Acquisition acquisition = Acquisition.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();
    private final Timer timer = new Timer();
    private final BooleanSupplier wiggleAcquisitionSupplier;
    private boolean isUpToSpeed;
    private double startingPivotDegrees = acquisition.getPivotPosition();

    public ShootCommand() {
        this.wiggleAcquisitionSupplier = () -> false;
        addRequirements(acquisition, kicker, shooter);
    }

    public ShootCommand(BooleanSupplier wiggleAcquisitionSupplier) {
        this.wiggleAcquisitionSupplier = wiggleAcquisitionSupplier;
        addRequirements(acquisition, kicker, shooter, swagLights);
    }

    @Override
    public void initialize() {
        timer.restart();
        isUpToSpeed = false;
        acquisition.setPivot(AcquisitionSetpoint.LOWERED);
    }

    @Override
    public void execute() {
        boolean validPosition = false;

        if (dashboard.isManualModeEnabled()) {
            double targetRPM = dashboard.getManualShooterRPM();

            shooter.getNearShooter().start(targetRPM * ShooterConstants.NEAR_SHOOTER_PERCENTAGE);
            shooter.getFarShooter().start(targetRPM);
            validPosition = true;
        } else {
            Pose2d robotPose = driveTrain.getState().Pose;
            double distance = shooter.getFarShooter().getTargetDistance(robotPose);

            for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
                if (formula.getMin() <= distance && formula.getMax() >= distance) {
                    double targetRPM = formula.getShooterRPM(distance);
                    shooter.getFarShooter().start(targetRPM);
                    shooter.getNearShooter()
                            .start(targetRPM * ShooterConstants.NEAR_SHOOTER_PERCENTAGE);
                    validPosition = true;
                    break;
                }
            }
            if (!validPosition) {
                swagLights.setTooCloseState();
            } else {
                swagLights.setDefaultState();
            }
        }

        if (validPosition && timer.hasElapsed(HOOD_SERVO_MOVE_TIME)) {
            // if (isUpToSpeed) {
            kicker.start();
            acquisition.acquire();
            if (wiggleAcquisitionSupplier.getAsBoolean()) {
                if (timer.get() % 1.5 <= 0.75) {
                    acquisition.setPivotDegrees(startingPivotDegrees + dashboard.getAcquisitionMinWiggle());
                } else {
                    acquisition.setPivotDegrees(startingPivotDegrees + dashboard.getAcquisitionMaxWiggle());
                }
                // }
            } else {
                isUpToSpeed = shooter.getNearShooter().isAtTargetRPM() &&
                        shooter.getFarShooter().isAtTargetRPM();
            }
        } else {
            kicker.stop();
            acquisition.stopIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.getFarShooter().stop();
        shooter.getNearShooter().stop();
        kicker.stop();
        acquisition.stopIntake();
        timer.stop();
        acquisition.setPivot(AcquisitionSetpoint.LOWERED);
    }
}
