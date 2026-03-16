package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHoods;

public class AdjustHoodsCommand extends Command {
    private final ShooterHoods shooterHoods = ShooterHoods.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();

    public AdjustHoodsCommand() {
        addRequirements(shooterHoods);
    }

    @Override
    public void execute() {
        if (dashboard.isManualModeEnabled()) {
            shooterHoods.getNearHood().setAngle(ShooterConstants.MANUAL_SHOOTER_ANGLE_DEG);
            shooterHoods.getFarHood().setAngle(ShooterConstants.MANUAL_SHOOTER_ANGLE_DEG);
        } else {
            Pose2d robotPose = driveTrain.getState().Pose;
            double distance = shooter.getFarShooter().getHubDistance(robotPose);

            for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
                if (formula.getMin() <= distance && formula.getMax() >= distance) {
                    double angle = formula.getAngle();
                    SmartDashboard.putNumber(DashboardConstants.ANGLE, angle);
                    shooterHoods.getFarHood().setAngle(angle);
                    shooterHoods.getNearHood().setAngle(angle);
                    break;
                }
            }
        }
    }
}
