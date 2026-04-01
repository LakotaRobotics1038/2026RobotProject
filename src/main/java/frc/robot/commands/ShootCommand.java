package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Acquisition;
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
    private static final double ACQUISITION_TILT_TIME = 1 + HOOD_SERVO_MOVE_TIME;

    private final AcquisitionPivot pivot = AcquisitionPivot.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Timer timer = new Timer();
    private final Function<Timer, Boolean> tiltAcquisitionSupplier;

    public ShootCommand() {
        this((timer) -> timer.hasElapsed(ACQUISITION_TILT_TIME));
    }

    public ShootCommand(Function<Timer, Boolean> tiltAcquisitionSupplier) {
        this.tiltAcquisitionSupplier = tiltAcquisitionSupplier;
        addRequirements(pivot, kicker, shooter, indexer, acquisition);
    }

    @Override
    public void initialize() {
        timer.restart();
        pivot.setAngleSetpoint(PivotSetpoint.LOWERED);
    }

    @Override
    public void execute() {
        boolean validPosition = false;

        if (Dashboard.MANUAL_MODE_ENABLED.get()) {
            double targetRPM = Dashboard.MANUAL_SHOOTER_RPM.get();

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
            kicker.start();
            indexer.start();
            acquisition.intake();
            if (tiltAcquisitionSupplier.apply(timer)) {
                pivot.setAngle(Dashboard.ACQUISITION_TILT.get());
            } else {
                pivot.setAngleSetpoint(PivotSetpoint.LOWERED);
            }
        } else {
            kicker.stop();
            indexer.stop();
            acquisition.stop();
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
        acquisition.stop();
        pivot.setAngleSetpoint(PivotSetpoint.LOWERED);
        if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
            swagLights.setOperatorState(OperatorStates.Default);
        }
    }
}
