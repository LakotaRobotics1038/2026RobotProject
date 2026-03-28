package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHoods;
import frc.robot.utils.dashboard.DashboardValue;

public class AdjustHoodsCommand extends Command {
    private final ShooterHoods shooterHoods = ShooterHoods.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    public AdjustHoodsCommand() {
        addRequirements(shooterHoods);
    }

    @Override
    public void execute() {
        if (DashboardValue.MANUAL_MODE_ENABLED.get()) {
            double angle = DashboardValue.MANUAL_SHOOTER_HOOD_ANGLE.get();
            shooterHoods.getNearHood().setAngle(angle);
            shooterHoods.getFarHood().setAngle(angle);
        } else {
            Pose2d robotPose = driveTrain.getState().Pose;
            double distance = shooter.getFarShooter().getTargetDistance(robotPose);

            for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
                if (formula.getMin() <= distance && formula.getMax() >= distance) {
                    double angle = formula.getAngle();
                    shooterHoods.getFarHood().setAngle(angle);
                    shooterHoods.getNearHood().setAngle(angle);
                    break;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
