package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ManualShootCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    public ManualShootCommand() {
        addRequirements(acquisition, kicker, shooter);
    }

    @Override
    public void execute() {
        double targetRPM = dashboard.getManualShooterRPM();

        shooter.getNearShooter().setAngle(ShooterConstants.MANUAL_SHOOTER_ANGLE_DEG);
        shooter.getNearShooter().start(targetRPM);

        shooter.getFarShooter().setAngle(ShooterConstants.MANUAL_SHOOTER_ANGLE_DEG);
        shooter.getFarShooter().start(targetRPM);

        kicker.start(KickerConstants.MANUAL_KICKER_RPM);

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
}
