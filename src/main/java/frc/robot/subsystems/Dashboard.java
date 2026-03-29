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
import frc.robot.utils.dashboard.DashboardValue;
import frc.robot.utils.dashboard.EntryDashboardValue;
import frc.robot.utils.dashboard.SendableDashboardValue;
import frc.robot.utils.dashboard.SuppliedDashboardValue;

public class Dashboard extends SubsystemBase {
    // Singleton Setup
    private static Dashboard instance;

    public static final SuppliedDashboardValue<Double> ROBOT_X = new SuppliedDashboardValue<>(
            "X Pos",
            () -> DriveTrain.getInstance().getX(),
            0.0);
    public static final SuppliedDashboardValue<Double> ROBOT_Y = new SuppliedDashboardValue<>(
            "Y Pos",
            () -> DriveTrain.getInstance().getY(),
            0.0);
    public static final SuppliedDashboardValue<Double> ROBOT_ROT = new SuppliedDashboardValue<>(
            "Rot",
            () -> DriveTrain.getInstance().getRotation(),
            0.0);
    public static final DashboardValue<Boolean> HUB_ALIGNED = new DashboardValue<>(
            "Hub Aligned",
            false);
    public static final DashboardValue<Boolean> MANUAL_MODE_ENABLED = new DashboardValue<>(
            "Manual Mode",
            false);
    public static final EntryDashboardValue<Double> MANUAL_SHOOTER_RPM = new EntryDashboardValue<>(
            "Manual Shoot RPM",
            v -> MathUtil.clamp(v,
                    ShooterConstants.MANUAL_SHOOTER_MIN_RPM,
                    ShooterConstants.MANUAL_SHOOTER_MAX_RPM),
            ShooterConstants.MANUAL_SHOOTER_RPM);
    public static final EntryDashboardValue<Double> MANUAL_SHOOTER_HOOD_ANGLE = new EntryDashboardValue<>(
            "Manual Shooter Hood Angle",
            v -> MathUtil.clamp(v,
                    ShooterHoodsConstants.SHOOTER_NO_RETRACTION_ANGLE,
                    ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE),
            ShooterHoodsConstants.MANUAL_SHOOTER_DEFAULT_ANGLE);
    public static final DashboardValue<Double> ACQUISITION_MIN_WIGGLE = new DashboardValue<>(
            "Acquisition Min Wiggle",
            AcquisitionConstants.PIVOT_MIN_WIGGLE);
    public static final DashboardValue<Double> ACQUISITION_MAX_WIGGLE = new DashboardValue<>(
            "Acquisition Max Wiggle",
            AcquisitionConstants.PIVOT_MAX_WIGGLE);
    public static final SendableDashboardValue<Field2d> FIELD = new SendableDashboardValue<>(
            "Field",
            v -> v.setRobotPose(
                    DriveTrain.getInstance().getState().Pose),
            new Field2d());
    public static final DashboardValue<SendableChooser<AutonChoices>> AUTO_CHOOSER = new DashboardValue<>(
            "Auton Choices",
            new SendableChooser<>());
    public static final DashboardValue<SendableChooser<Double>> DELAY_CHOOSER = new DashboardValue<>(
            "Delay Choices",
            new SendableChooser<>());

    public static Dashboard getInstance() {
        if (instance == null) {
            System.out.println("Creating a new Dashboard");
            instance = new Dashboard();
        }
        return instance;
    }

    private Dashboard() {
        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            SmartDashboard.putNumber(DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope());
            SmartDashboard.putNumber(DashboardConstants.shooterYInterceptKey(formula.getAngle()),
                    formula.getYIntercept());
        }

        Field2d field = FIELD.get();
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> field.getObject("target pose").setPose(pose));
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("poses").setPoses(poses));
    }

    @Override
    public void periodic() {
        for (DashboardValue<?> value : DashboardValue.values()) {
            value.periodic();
        }

        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            formula.setSlope(SmartDashboard.getNumber(
                    DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope()));
            formula.setYIntercept(SmartDashboard.getNumber(
                    DashboardConstants.shooterYInterceptKey(formula.getAngle()), formula.getYIntercept()));
        }
    }

    public void clearFieldTrajectory() {
        Dashboard.FIELD.get().getObject("traj").setPoses(new ArrayList<>());
    }
}
