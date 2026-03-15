package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
    private boolean isUpToSpeed = false;
    private final Timer timer = new Timer();

    public ShootCommand() {
        addRequirements(acquisition, kicker, shooter);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if (dashboard.isManualModeEnabled()) {
            double targetRPM = dashboard.getManualShooterRPM();

            shooter.getNearShooter().setAngle(ShooterConstants.MANUAL_SHOOTER_ANGLE_DEG);
            shooter.getNearShooter().start(targetRPM * ShooterConstants.NEAR_SHOOTER_PERCENTAGE);

            shooter.getFarShooter().setAngle(ShooterConstants.MANUAL_SHOOTER_ANGLE_DEG);
            shooter.getFarShooter().start(targetRPM);
        } else {
            Pose2d robotPose = driveTrain.getState().Pose;
            double farDistance = shooter.getFarShooter().getHubDistance(robotPose);
            double nearDistance = shooter.getNearShooter().getHubDistance(robotPose);

            // SmartDashboard.putNumber("FAR", farShooterHubDistance);
            // SmartDashboard.putNumber("NEAR", nearShooterHubDistance);
            // double minDistance = Math.min(nearShooterHubDistance, farShooterHubDistance);
            // double maxDistance = Math.max(nearShooterHubDistance, farShooterHubDistance);
            for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
                // if (formula.getMin() <= minDistance && formula.getMax() >= maxDistance) {
                shooter.getNearShooter().setAngle(formula.getAngle());
                shooter.getNearShooter().start(formula.getShooterRPM(nearDistance));
                shooter.getFarShooter().setAngle(formula.getAngle());
                shooter.getFarShooter().start(formula.getShooterRPM(farDistance));
                break;
                // }
            }
        }

        if (timer.hasElapsed(HOOD_SERVO_MOVE_TIME)) {
            if (isUpToSpeed) {
                kicker.start();
                acquisition.acquire();
                if (timer.get() >= 4) {
                    acquisition.setPivot(AcquisitionSetpoint.RAISED);
                } else if (timer.get() >= 1.5) {
                    if (timer.get() % 1 <= 0.5) {
                        acquisition.setPivot(AcquisitionSetpoint.LOW_RAISE);
                    } else {
                        acquisition.setPivot(AcquisitionSetpoint.HIGH_RAISE);
                    }
                }
                // if (timer.get() % 2 <= 0.1) {
                // acquisition.stopIntake();
                // // acquisition.acquireSlow();
                // } else {
                // acquisition.acquire();
                // }
            } else {
                isUpToSpeed = shooter.getNearShooter().isAtTargetRPM() &&
                        shooter.getFarShooter().isAtTargetRPM();
                kicker.stop();
                acquisition.stopIntake();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.getFarShooter().stop();
        shooter.getNearShooter().stop();
        kicker.stop();
        acquisition.stopIntake();
        acquisition.setPivot(AcquisitionSetpoint.LOWERED);
        timer.stop();
        timer.reset();
    }
}
