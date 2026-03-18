package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private static final double HOOD_SERVO_MOVE_TIME = 0.5;

    private final Acquisition acquisition = Acquisition.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();
    private final Timer timer = new Timer();
    private final BooleanSupplier wiggleAcquisitionSupplier;
    private boolean isUpToSpeed;

    public ShootCommand() {
        this.wiggleAcquisitionSupplier = () -> false;
        addRequirements(acquisition, kicker, shooter);
    }

    public ShootCommand(BooleanSupplier wiggleAcquisitionSupplier) {
        this.wiggleAcquisitionSupplier = wiggleAcquisitionSupplier;
        addRequirements(acquisition, kicker, shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
        isUpToSpeed = false;
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
            Translation2d virtualHub = Shooter.getVirtualHubPosition(robotPose, driveTrain.getState().Speeds);
            double distance = shooter.getFarShooter().getHubDistance(robotPose, virtualHub);

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
        }

        if (validPosition && timer.hasElapsed(HOOD_SERVO_MOVE_TIME)) {
            if (isUpToSpeed) {
                kicker.start();
                acquisition.acquire();
                if (wiggleAcquisitionSupplier.getAsBoolean()) {
                    if (timer.get() % 1 <= 0.5) {
                        acquisition.setPivot(AcquisitionSetpoint.LOW_RAISE);
                    } else {
                        acquisition.setPivot(AcquisitionSetpoint.HIGH_RAISE);
                    }
                }
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
    }
}
