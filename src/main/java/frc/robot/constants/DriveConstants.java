package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(1, 1, 1);

    public static final double DEFAULT_MAX_POWER = 1.0;
    public static final double BUMP_SLOWDOWN_POWER = 0.3;

    public static final double BUMP_APPROACH_SPEED_THRESHOLD = 0.5;

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.525); // Inches
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(23.525); // Inches
    private static final double ROBOT_MAX_LENGTH = Math.max(TRACK_WIDTH, WHEEL_BASE);
    public static final double ROBOT_SIZE_RADIUS = Math
            .sqrt(Math.pow(ROBOT_MAX_LENGTH, 2) + Math.pow(ROBOT_MAX_LENGTH, 2)) / 2;

    public static final double FINE_ADJUSTMENT_PERCENT = 0.2;

    // SPEED_AT_12_VOLTS desired top speed
    public static final double MAX_SPEED = SwerveConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    // 3/4 of a rotation angular velocity per second max
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private static final double P = 8.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double ALIGNMENT_TOLERANCE_RAD = Math.toRadians(5.0);
    public static final double MAX_ROTATION_POWER = 1.0;
    public static final PIDController ROTATION_CONTROLLER = new PIDController(P, I, D);
    static {
        ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        ROTATION_CONTROLLER.setTolerance(ALIGNMENT_TOLERANCE_RAD);
    }
}