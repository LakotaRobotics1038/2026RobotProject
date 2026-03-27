package frc.robot.constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autons.AutonSelector.AutonChoices;
import frc.robot.subsystems.Dashboard.DashboardValue;
import frc.robot.subsystems.Dashboard.EntryDashboardValue;
import frc.robot.subsystems.Dashboard.SendableDashboardValue;
import frc.robot.subsystems.Dashboard.SimpleDashboardValue;
import frc.robot.subsystems.Dashboard.SuppliedDashboardValue;
import frc.robot.subsystems.DriveTrain;

public final class DashboardConstants {
    public static final DashboardValue<Double> ROBOT_X = new SuppliedDashboardValue<>(
            "X Pos",
            () -> DriveTrain.getInstance().getX(),
            0.0);
    public static final DashboardValue<Double> ROBOT_Y = new SuppliedDashboardValue<>(
            "Y Pos",
            () -> DriveTrain.getInstance().getY(),
            0.0);
    public static final DashboardValue<Double> ROBOT_ROT = new SuppliedDashboardValue<>(
            "Rot",
            () -> DriveTrain.getInstance().getRotation(),
            0.0);
    public static final DashboardValue<Boolean> HUB_ALIGNED = new EntryDashboardValue<>(
            "Hub Aligned",
            false);
    public static final DashboardValue<Boolean> MANUAL_MODE_ENABLED = new EntryDashboardValue<>(
            "Manual Mode",
            false);
    public static final DashboardValue<Double> MANUAL_SHOOTER_RPM = new EntryDashboardValue<>(
            "Manual Shoot RPM",
            v -> MathUtil.clamp(v,
                    ShooterConstants.MANUAL_SHOOTER_MIN_RPM,
                    ShooterConstants.MANUAL_SHOOTER_MAX_RPM),
            ShooterConstants.MANUAL_SHOOTER_RPM);
    public static final DashboardValue<Double> MANUAL_SHOOTER_HOOD_ANGLE = new EntryDashboardValue<>(
            "Manual Shooter Hood Angle",
            v -> MathUtil.clamp(v,
                    ShooterHoodsConstants.SHOOTER_NO_RETRACTION_ANGLE,
                    ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE),
            ShooterHoodsConstants.MANUAL_SHOOTER_DEFAULT_ANGLE);
    public static final DashboardValue<Double> ACQUISITION_MIN_WIGGLE = new EntryDashboardValue<>(
            "Acquisition Min Wiggle",
            AcquisitionConstants.PIVOT_MIN_WIGGLE);
    public static final DashboardValue<Double> ACQUISITION_MAX_WIGGLE = new EntryDashboardValue<>(
            "Acquisition Max Wiggle",
            AcquisitionConstants.PIVOT_MAX_WIGGLE);
    public static final DashboardValue<Field2d> FIELD = new SendableDashboardValue<>(
            new Field2d(),
            v -> v.setRobotPose(
                    DriveTrain.getInstance().getState().Pose));
    public static final DashboardValue<SendableChooser<AutonChoices>> AUTO_CHOOSER = new SimpleDashboardValue<>(
            "Auton Choices",
            new SendableChooser<>());
    public static final DashboardValue<SendableChooser<Double>> DELAY_CHOOSER = new SimpleDashboardValue<>(
            "Delay Choices",
            new SendableChooser<>());

    public static String shooterSlopeKey(double angle) {
        return "Shooter Slope " + (int) angle;
    }

    public static String shooterYInterceptKey(double angle) {
        return "Shooter Y Intercept " + (int) angle;
    }
}
