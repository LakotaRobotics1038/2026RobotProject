package frc.robot.subsystems;

import java.util.ArrayList;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autons.AutonSelector.AutonChoices;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterHoodsConstants;

public class Dashboard extends SubsystemBase {
    // Inputs
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    // Choosers
    private final SendableChooser<AutonChoices> autoChooser = new SendableChooser<>();
    private final SendableChooser<Double> delayChooser = new SendableChooser<>();

    // Variables
    private final Field2d field = new Field2d();
    private boolean hubAligned = false;
    private boolean manualModeEnabled = false;
    private double manualShooterRPM = ShooterConstants.MANUAL_SHOOTER_RPM;
    private double manualShooterHoodAngle = ShooterHoodsConstants.MANUAL_SHOOTER_DEFAULT_ANGLE;
    private double acquisitionMinWiggle = AcquisitionConstants.PIVOT_MIN_WIGGLE;
    private double acquisitionMaxWiggle = AcquisitionConstants.PIVOT_MAX_WIGGLE;

    // Singleton Setup
    private static Dashboard instance;

    public static Dashboard getInstance() {
        if (instance == null) {
            System.out.println("Creating a new Dashboard");
            instance = new Dashboard();
        }
        return instance;
    }

    private Dashboard() {
        SmartDashboard.putData(DashboardConstants.AUTON_CHOICES, autoChooser);
        SmartDashboard.putData(DashboardConstants.DELAY_CHOICES, delayChooser);
        SmartDashboard.putBoolean(DashboardConstants.MANUAL_MODE_ENABLED, manualModeEnabled);
        SmartDashboard.putNumber(DashboardConstants.MANUAL_SHOOTER_RPM, manualShooterRPM);
        SmartDashboard.putNumber(DashboardConstants.MANUAL_SHOOTER_HOOD_ANGLE, manualShooterHoodAngle);
        SmartDashboard.putNumber(DashboardConstants.ACQUISITION_MIN_WIGGLE, acquisitionMinWiggle);
        SmartDashboard.putNumber(DashboardConstants.ACQUISITION_MAX_WIGGLE, acquisitionMaxWiggle);
        SmartDashboard.putData(field);

        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            SmartDashboard.putNumber(DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope());
            SmartDashboard.putNumber(DashboardConstants.shooterYInterceptKey(formula.getAngle()),
                    formula.getYIntercept());
        }

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> field.getObject("target pose").setPose(pose));

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("poses").setPoses(poses));
    }

    @Override
    public void periodic() {
        // Controls Tab
        manualModeEnabled = SmartDashboard.getBoolean(DashboardConstants.MANUAL_MODE_ENABLED, manualModeEnabled);
        manualShooterRPM = MathUtil.clamp(
                SmartDashboard.getNumber(DashboardConstants.MANUAL_SHOOTER_RPM, manualShooterRPM),
                ShooterConstants.MANUAL_SHOOTER_MIN_RPM,
                ShooterConstants.MANUAL_SHOOTER_MAX_RPM);
        manualShooterHoodAngle = MathUtil.clamp(
                SmartDashboard.getNumber(DashboardConstants.MANUAL_SHOOTER_HOOD_ANGLE, manualShooterHoodAngle),
                ShooterHoodsConstants.SHOOTER_NO_RETRACTION_ANGLE,
                ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE);
        acquisitionMinWiggle = SmartDashboard.getNumber(DashboardConstants.ACQUISITION_MIN_WIGGLE,
                acquisitionMinWiggle);
        acquisitionMaxWiggle = SmartDashboard.getNumber(DashboardConstants.ACQUISITION_MAX_WIGGLE,
                acquisitionMaxWiggle);

        SmartDashboard.putNumber(DashboardConstants.ROBOT_X, driveTrain.getX());
        SmartDashboard.putNumber(DashboardConstants.ROBOT_Y, driveTrain.getY());
        SmartDashboard.putNumber(DashboardConstants.ROBOT_ROT, driveTrain.getRotation());
        SmartDashboard.putBoolean(DashboardConstants.HUB_ALIGNED, hubAligned);
        SmartDashboard.putNumber(DashboardConstants.MANUAL_SHOOTER_RPM, manualShooterRPM);
        SmartDashboard.putNumber(DashboardConstants.MANUAL_SHOOTER_HOOD_ANGLE, manualShooterHoodAngle);

        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            formula.setSlope(SmartDashboard.getNumber(
                    DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope()));
            formula.setYIntercept(SmartDashboard.getNumber(
                    DashboardConstants.shooterYInterceptKey(formula.getAngle()), formula.getYIntercept()));
        }

        field.setRobotPose(driveTrain.getState().Pose);
    }

    /**
     * Removes the trajectory line from the dashboard
     */
    public void clearTrajectory() {
        this.field.getObject("traj").setPoses(new ArrayList<>());
    }

    /**
     * Gets the sendable chooser for Auton Modes
     *
     * @return The sendable chooser
     */
    public SendableChooser<AutonChoices> getAutoChooser() {
        return autoChooser;
    }

    public SendableChooser<Double> getDelayChooser() {
        return delayChooser;
    }

    public void setHubAligned(boolean hubAligned) {
        this.hubAligned = hubAligned;
    }

    public boolean isManualModeEnabled() {
        return manualModeEnabled;
    }

    public double getManualShooterRPM() {
        return manualShooterRPM;
    }

    public double getManualShooterHoodAngle() {
        return manualShooterHoodAngle;
    }

    public void nudgeManualShooterRPMBackward() {
        setManualShooterRPM(manualShooterRPM - ShooterConstants.MANUAL_SHOOTER_RPM_STEP);
    }

    public void nudgeManualShooterRPMForward() {
        setManualShooterRPM(manualShooterRPM + ShooterConstants.MANUAL_SHOOTER_RPM_STEP);
    }

    public void resetManualShooterRPM() {
        setManualShooterRPM(ShooterConstants.MANUAL_SHOOTER_RPM);
    }

    public void nudgeManualShooterHoodAngleBackward() {
        manualShooterHoodAngle -= ShooterHoodsConstants.MANUAL_SHOOTER_ANGLE_INCREMENT;
    }

    public void nudgeManualShooterHoodAngleForward() {
        manualShooterHoodAngle += ShooterHoodsConstants.MANUAL_SHOOTER_ANGLE_INCREMENT;
    }

    public void resetManualShooterHoodAngle() {
        manualShooterHoodAngle = ShooterHoodsConstants.MANUAL_SHOOTER_DEFAULT_ANGLE;
    }

    public double getAcquisitionMinWiggle() {
        return acquisitionMinWiggle;
    }

    public double getAcquisitionMaxWiggle() {
        return acquisitionMaxWiggle;
    }

    private void setManualShooterRPM(double manualShooterRPM) {
        this.manualShooterRPM = MathUtil.clamp(
                manualShooterRPM,
                ShooterConstants.MANUAL_SHOOTER_MIN_RPM,
                ShooterConstants.MANUAL_SHOOTER_MAX_RPM);
        SmartDashboard.putNumber(DashboardConstants.MANUAL_SHOOTER_RPM, this.manualShooterRPM);
    }
}
