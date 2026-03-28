package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.SwagLights.OperatorStates;

public class ShootCommand extends Command {
    private final Indexer indexer = Indexer.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();

    public ShootCommand() {
        addRequirements(indexer, kicker, shooter);
    }

    @Override
    public void execute() {
        boolean validPosition = false;

        if (dashboard.isManualModeEnabled()) {
            double targetRPM = dashboard.getManualShooterRPM();

            shooter.start(targetRPM);
            validPosition = true;
            if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
                swagLights.setOperatorState(OperatorStates.Default);
            }
        } else {
            Pose2d robotPose = driveTrain.getState().Pose;
            double distance = shooter.getTargetDistance(robotPose);

            for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
                if (formula.getMin() <= distance && formula.getMax() >= distance) {
                    double targetRPM = formula.getShooterRPM(distance);
                    shooter.start(targetRPM);
                    validPosition = true;
                    break;
                }
            }
            if (!validPosition) {
                swagLights.setOperatorState(OperatorStates.TooClose);
            } else if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
                swagLights.setOperatorState(OperatorStates.Default);
            }
        }

        if (validPosition) {
            kicker.start();
            indexer.in();
        } else {
            kicker.stop();
            indexer.stop();
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
        indexer.stop();
        if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
            swagLights.setOperatorState(OperatorStates.Default);
        }
    }
}
