package frc.robot.subsystems;

import java.util.ArrayList;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autons.AutonSelector.AutonChoices;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.ShooterConstants;

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
        SmartDashboard.putData(field);

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

        SmartDashboard.putNumber(DashboardConstants.ROBOT_X, driveTrain.getX());
        SmartDashboard.putNumber(DashboardConstants.ROBOT_Y, driveTrain.getY());
        SmartDashboard.putNumber(DashboardConstants.ROBOT_ROT, driveTrain.getRotation());
        SmartDashboard.putBoolean(DashboardConstants.HUB_ALIGNED, hubAligned);
        SmartDashboard.putNumber(DashboardConstants.MANUAL_SHOOTER_RPM, manualShooterRPM);

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

    public void nudgeManualShooterRPM(double deltaRPM) {
        setManualShooterRPM(manualShooterRPM + deltaRPM);
    }

    public void resetManualShooterRPM() {
        setManualShooterRPM(ShooterConstants.MANUAL_SHOOTER_RPM);
    }

    private void setManualShooterRPM(double manualShooterRPM) {
        this.manualShooterRPM = MathUtil.clamp(
                manualShooterRPM,
                ShooterConstants.MANUAL_SHOOTER_MIN_RPM,
                ShooterConstants.MANUAL_SHOOTER_MAX_RPM);
        SmartDashboard.putNumber(DashboardConstants.MANUAL_SHOOTER_RPM, this.manualShooterRPM);
    }
}
