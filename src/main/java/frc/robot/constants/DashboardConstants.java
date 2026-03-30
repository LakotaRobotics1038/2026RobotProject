package frc.robot.constants;

public final class DashboardConstants {
    public static final String AUTON_CHOICES = "Auton Choices";
    public static final String DELAY_CHOICES = "Delay Choices";

    public static final String ROBOT_X = "X Pos";
    public static final String ROBOT_Y = "Y Pos";
    public static final String ROBOT_ROT = "Rot";
    public static final String HUB_ALIGNED = "Hub Aligned";
    public static final String MANUAL_MODE_ENABLED = "Manual Mode";
    public static final String MANUAL_SHOOTER_RPM = "Manual Shoot RPM";
    public static final String MANUAL_SHOOTER_HOOD_ANGLE = "Manual Shooter Hood Angle";
    public static final String ACQUISITION_TILT = "Acquisition Tilt";
    public static final String ENABLE_ACQUISITION_PIVOT = "Enable Acquisition Pivot";

    public static String shooterSlopeKey(double angle) {
        return "Shooter Slope " + (int) angle;
    }

    public static String shooterYInterceptKey(double angle) {
        return "Shooter Y Intercept " + (int) angle;
    }
}
