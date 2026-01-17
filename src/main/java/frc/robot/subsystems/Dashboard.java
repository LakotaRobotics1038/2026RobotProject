package frc.robot.subsystems;

import java.util.ArrayList;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autons.AutonSelector;
import frc.robot.constants.DashboardConstants;

public class Dashboard extends SubsystemBase {
    // Inputs
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    // Choosers
    private final SendableChooser<AutonSelector.AutonChoices> autoChooser = new SendableChooser<>();

    // Variables
    private final Field2d field = new Field2d();

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
        SmartDashboard.putNumber(DashboardConstants.DELAY_CHOICES, 0);

        SmartDashboard.putData(field);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> field.getObject("target pose").setPose(pose));

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("poses").setPoses(poses));
    }

    @Override
    public void periodic() {
        // Controls Tab

        SmartDashboard.putNumber(DashboardConstants.ROBOT_X, driveTrain.getX());
        SmartDashboard.putNumber(DashboardConstants.ROBOT_Y, driveTrain.getY());
        SmartDashboard.putNumber(DashboardConstants.ROBOT_ROT, driveTrain.getRotation());

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
    public SendableChooser<AutonSelector.AutonChoices> getAutoChooser() {
        return autoChooser;
    }

    public double getDelay() {
        return SmartDashboard.getNumber(DashboardConstants.DELAY_CHOICES, 0);
    }
}
