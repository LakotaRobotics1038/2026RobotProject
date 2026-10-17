package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterValue;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

public class AdjustHoodCommand extends Command {
    private final ShooterHood shooterHood = ShooterHood.getInstance();
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
            double distance = Shooter.getTargetDistance(driveTrain.getState().Pose);
            ShooterValue shooterValue = ShooterConstants.SHOOTER_RPM_MAP.get(distance);
            shooterHood.setAngle(shooterValue.angle());
        }
        shooterHood.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
