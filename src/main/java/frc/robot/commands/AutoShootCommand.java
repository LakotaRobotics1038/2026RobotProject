package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class AutoShootCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    public AutoShootCommand() {
        addRequirements(acquisition, kicker, shooter);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveTrain.getState().Pose;
        double farDistance = shooter.getFarShooter().getHubDistance(robotPose);
        double nearDistance = shooter.getNearShooter().getHubDistance(robotPose);

        autoShoot(nearDistance, farDistance);
        if (shooter.getNearShooter().isAtTargetRPM() && shooter.getFarShooter().isAtTargetRPM()) {
            acquisition.acquire();
        } else {
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
    }

    /**
     * Sets the shooter to a certain speed given the distance to the hub. Assumes
     * that it is already aligned.
     * If the robot is too close or too far for any of the angles, it silently
     * fails. If distances overlap, lesser
     * angles will be preferred.
     *
     * @param nearShooterHubDistance The distance from the near shooter module to
     *                               the hub.
     * @param farShooterHubDistance  The distance from the far shooter module to the
     *                               hub.
     */
    private void autoShoot(double nearShooterHubDistance, double farShooterHubDistance) {
        double minDistance = Math.min(nearShooterHubDistance, farShooterHubDistance);
        double maxDistance = Math.max(nearShooterHubDistance, farShooterHubDistance);
        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            if (formula.getMin() <= minDistance && formula.getMax() >= maxDistance) {
                shooter.getNearShooter().setAngle(formula.getAngle());
                shooter.getNearShooter().start(formula.getShooterRPM(nearShooterHubDistance));
                shooter.getFarShooter().setAngle(formula.getAngle());
                shooter.getFarShooter().start(formula.getShooterRPM(farShooterHubDistance));
                kicker.start();
                break;
            }
        }
    }
}
