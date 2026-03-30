package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.AcquisitionPivot;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.SwagLights.OperatorStates;

public class ShootCommand extends Command {
    private static final double HOOD_SERVO_MOVE_TIME = 0.5;

    private final AcquisitionPivot pivot = AcquisitionPivot.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();
    private final Timer timer = new Timer();
    private final BooleanSupplier tiltAcquisitionSupplier;
    private boolean isUpToSpeed;

    public ShootCommand() {
        this.tiltAcquisitionSupplier = () -> false;
        addRequirements(pivot, kicker, shooter, indexer);
    }

    public ShootCommand(BooleanSupplier tiltAcquisitionSupplier) {
        this.tiltAcquisitionSupplier = tiltAcquisitionSupplier;
        addRequirements(pivot, kicker, shooter, indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
        isUpToSpeed = false;
        pivot.setAngleSetpoint(PivotSetpoint.LOWERED);
    }

    @Override
    public void execute() {
        boolean validPosition = false;

        if (dashboard.isManualModeEnabled()) {
            double targetRPM = dashboard.getManualShooterRPM();

            shooter.getNearShooter().start(targetRPM *
                    ShooterConstants.NEAR_SHOOTER_PERCENTAGE);
            shooter.getFarShooter().start(targetRPM);
            validPosition = true;
            if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
                swagLights.setOperatorState(OperatorStates.Default);
            }
        } else {
            Pose2d robotPose = driveTrain.getState().Pose;
            double distance = shooter.getFarShooter().getTargetDistance(robotPose);

            for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
                if (formula.getMin() <= distance && formula.getMax() >= distance) {
                    double targetRPM = formula.getShooterRPM(distance);
                    shooter.getFarShooter().start(targetRPM);
                    shooter.getNearShooter()
                            .start(targetRPM * ShooterConstants.NEAR_SHOOTER_PERCENTAGE);
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

        if (validPosition && timer.hasElapsed(HOOD_SERVO_MOVE_TIME)) {
            if (isUpToSpeed) {
                kicker.start();
                indexer.start();
                if (tiltAcquisitionSupplier.getAsBoolean()) {
                    pivot.setAngle(dashboard.getAcquisitionTilt());
                }
            } else {
                isUpToSpeed = shooter.getNearShooter().isAtTargetRPM() &&
                        shooter.getFarShooter().isAtTargetRPM();
            }
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
        shooter.getFarShooter().stop();
        shooter.getNearShooter().stop();
        kicker.stop();
        indexer.stop();
        timer.stop();
        pivot.setAngleSetpoint(PivotSetpoint.LOWERED);
        if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
            swagLights.setOperatorState(OperatorStates.Default);
        }
    }
}
