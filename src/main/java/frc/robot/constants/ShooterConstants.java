package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.dashboard.DashboardValue;

public final class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_TOP_CAN_ID = 5;
    public static final int SHOOTER_MOTOR_LEFT_BOTTOM_CAN_ID = 3;
    public static final int SHOOTER_MOTOR_RIGHT_TOP_CAN_ID = 16;
    public static final int SHOOTER_MOTOR_RIGHT_BOTTOM_CAN_ID = 15;
    public static final Translation2d SHOOTER_BARREL_CENTER = new Translation2d(Units.inchesToMeters(-5.202363), 0);

    public static final double OPERATING_TOLERANCE = 300;

    public static final double P = 0.0002;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = 0.00187;
    public static final double A = 0.0;

    public static final double SHOOTER_DIRECTION_FROM_FORWARD_RAD = -Math.PI / 2.0;
    public static final double MANUAL_SHOOTER_RPM = 2900.0;
    public static final double MANUAL_SHOOTER_RPM_STEP = 50.0;
    public static final double MANUAL_SHOOTER_MIN_RPM = 2000.0;
    // Make sure there's a 0 at the end so manual mode goes by the shooter RPM step
    public static final double MANUAL_SHOOTER_MAX_RPM = (int) (NeoMotorConstants.VORTEX_FREE_SPEED_RPM
            / MANUAL_SHOOTER_RPM_STEP) * MANUAL_SHOOTER_RPM_STEP;

    public static final double MAX_SHOOTER_RPM = 4000.0;

    public static final InterpolatingShooterMap SHOOTER_RPM_MAP = new InterpolatingShooterMap();

    static {
        SHOOTER_RPM_MAP.put(Units.inchesToMeters(52), new ShooterValue(73, 2500));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(62), new ShooterValue(72, 2500));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(72), new ShooterValue(70, 2500));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(92), new ShooterValue(70, 2700));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(105), new ShooterValue(70, 2750));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(110), new ShooterValue(65, 2750));
        SHOOTER_RPM_MAP.put(Units.inchesToMeters(120), new ShooterValue(65, 2800));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(130), new ShooterValue(65, 2900));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(140), new ShooterValue(65, 3000));
        // SHOOTER_RPM_MAP.put(Units.inchesToMeters(158), new ShooterValue(60, 3100));
        SHOOTER_RPM_MAP.put(Units.inchesToMeters(188), new ShooterValue(60, 3500));
    }

    public record ShooterValue(double angle, double rpm) {
    }

    public static final class InterpolatingShooterMap extends InterpolatingTreeMap<Double, ShooterValue> {
        public InterpolatingShooterMap() {
            super(InverseInterpolator.forDouble(),
                    (ShooterValue startValue, ShooterValue endValue, double t) -> new ShooterValue(
                            MathUtil.interpolate(startValue.angle(), endValue.angle(), t),
                            MathUtil.interpolate(startValue.rpm(), endValue.rpm(), t)));
        }
    }
}
