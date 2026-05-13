package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterValue;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.SwagLights.OperatorStates;

public class ShooterCommand extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();

    public ShooterCommand() {
        addRequirements(shooter, kicker);
    }

    @Override
    public void execute() {
        if (Dashboard.MANUAL_MODE_ENABLED.get()) {
            double targetRPM = Dashboard.MANUAL_SHOOTER_RPM.get();

            shooter.start(targetRPM);
        } else {
            double distance = Shooter.getTargetDistance(driveTrain.getState().Pose);
            ShooterValue shooterValue = ShooterConstants.SHOOTER_RPM_MAP.get(distance);
            shooter.start(shooterValue.rpm());
        }
        kicker.start();
        if (shooter.isAtTargetRPM() && kicker.isAtTargetRPM()) {
            Dashboard.UP_TO_SPEED.set(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        kicker.stop();
        if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
            swagLights.setOperatorState(OperatorStates.Default);
        }
        Dashboard.UP_TO_SPEED.set(false);
    }
}
