package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.SwagLights.OperatorStates;

public class ShootCommand extends Command {
    private static final double HOOD_SERVO_MOVE_TIME = 0.5;
    private static final double ACQUISITION_LOWER_WIGGLE_TIME = 0.75;
    private static final double ACQUISITION_RAISE_WIGGLE_TIME = 0.75;

    private final Acquisition acquisition = Acquisition.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();
    private final Timer timer = new Timer();
    private final BooleanSupplier wiggleAcquisitionSupplier;
    private double startingPivotDegrees;

    public ShootCommand() {
        this.wiggleAcquisitionSupplier = () -> false;
        addRequirements(acquisition, kicker, shooter);
    }

    public ShootCommand(BooleanSupplier wiggleAcquisitionSupplier) {
        this.wiggleAcquisitionSupplier = wiggleAcquisitionSupplier;
        addRequirements(acquisition, kicker, shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
        startingPivotDegrees = acquisition.getPivotPosition();
        acquisition.setPivot(AcquisitionSetpoint.LOWERED);
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
            acquisition.acquire();
            if (wiggleAcquisitionSupplier.getAsBoolean()) {
                if (timer.get() % (ACQUISITION_LOWER_WIGGLE_TIME
                        + ACQUISITION_RAISE_WIGGLE_TIME) <= ACQUISITION_LOWER_WIGGLE_TIME) {
                    acquisition.setPivotDegrees(startingPivotDegrees + Dashboard.ACQUISITION_MIN_WIGGLE.get());
                } else {
                    acquisition.setPivotDegrees(startingPivotDegrees + Dashboard.ACQUISITION_MAX_WIGGLE.get());
                }
            }
        } else {
            kicker.stop();
            acquisition.stopIntake();
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
        acquisition.stopIntake();
        timer.stop();
        acquisition.setPivot(AcquisitionSetpoint.LOWERED);
        if (swagLights.getOperatorState() == SwagLights.OperatorStates.TooClose) {
            swagLights.setOperatorState(OperatorStates.Default);
        }
    }
}
