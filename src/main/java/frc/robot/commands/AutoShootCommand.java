package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

import java.util.List;

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
     * @param nearHubDistance The distance from the near shooter module to the hub.
     * @param farHubDistance The distance from the far shooter module to the hub.
     */
    private void autoShoot(double nearHubDistance, double farHubDistance) {
        for (AutoShootFormula formula : AutoShootCommand.AUTO_SHOOT_FORMULAS) {
            if (formula.getMin() <= nearHubDistance && formula.getMax() >= farHubDistance) {
                shooter.getNearShooter().setAngle(formula.getAngle());
                shooter.getNearShooter().start(formula.getShooterRPM(nearHubDistance));
                shooter.getFarShooter().setAngle(formula.getAngle());
                shooter.getFarShooter().start(formula.getShooterRPM(farHubDistance));
                kicker.start(formula.getKickerRPM(nearHubDistance)); // TODO Should this be based on near or far? Does it matter?
                break;
            }
        }
    }

    public static class AutoShootFormula {
        private final double shooterSlope;
        private final double shooterYIntercept;
        private final double kickerSlope;
        private final double kickerYIntercept;
        private final double min;
        private final double max;
        private final double angle;

        private AutoShootFormula(
                double angle,
                double shooterSlope,
                double shooterYIntercept,
                double kickerSlope,
                double kickerYIntercept,
                double min,
                double max) {
            this.angle = angle;
            this.shooterSlope = shooterSlope;
            this.shooterYIntercept = shooterYIntercept;
            this.kickerSlope = kickerSlope;
            this.kickerYIntercept = kickerYIntercept;
            this.min = min;
            this.max = max;
        }

        public double getAngle() {
            return angle;
        }

        public double getMin() {
            return min;
        }

        public double getMax() {
            return max;
        }

        public double getShooterRPM(double distance) {
            return shooterSlope * distance + shooterYIntercept;
        }

        public double getKickerRPM(double distance) {
            return kickerSlope * distance + kickerYIntercept;
        }
    }

    /**
     * List of angles and their corresponding shooter formulas. The formula is used
     * to calculate the RPM of the shooter
     * based on the distance to the target. The min and max values represent the
     * range of that angle.
     */
    public static final List<AutoShootFormula> AUTO_SHOOT_FORMULAS = List.of(
            new AutoShootFormula(
                    55.0,
                    454.97,
                    2033.9,
                    -547.33,
                    4856.8,
                    1.7,
                    3.25),
            new AutoShootFormula(
                    65.0,
                    448.4,
                    2056.9,
                    687.67,
                    862.02,
                    2.5,
                    5.1)
    );
}
