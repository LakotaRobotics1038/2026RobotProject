package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
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
    public void initialize() {
        Pose2d robotPose = driveTrain.getState().Pose;
        double farDistance = shooter.getFarShooter().getHubDistance(robotPose);
        double nearDistance = shooter.getNearShooter().getHubDistance(robotPose);

        shooter.getFarShooter().autoShoot(farDistance);
        shooter.getNearShooter().autoShoot(nearDistance);
    }

    @Override
    public void execute() {
        if (shooter.getNearShooter().isAtTargetRPM() && shooter.getFarShooter().isAtTargetRPM()) {
            acquisition.acquire();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.getFarShooter().stop();
        shooter.getNearShooter().stop();
        kicker.stop();
        acquisition.stopIntake();
    }
}
