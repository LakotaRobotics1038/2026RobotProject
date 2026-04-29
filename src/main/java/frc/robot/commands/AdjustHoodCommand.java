package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

public class AdjustHoodCommand extends Command {
    private final ShooterHood shooterHood = ShooterHood.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    public AdjustHoodCommand() {
        addRequirements(shooterHood);
    }

    @Override
    public void execute() {
        if (Dashboard.MANUAL_MODE_ENABLED.get()) {
            double angle = Dashboard.MANUAL_SHOOTER_HOOD_ANGLE.get();
            shooterHood.setAngle(angle);
        } else {
            Pose2d robotPose = driveTrain.getState().Pose;
            double distance = shooter.getTargetDistance(robotPose);

            for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
                if (formula.getMin() <= distance && formula.getMax() >= distance) {
                    double angle = formula.getAngle();
                    shooterHood.setAngle(angle);
                    break;
                }
            }
        }
        shooterHood.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
