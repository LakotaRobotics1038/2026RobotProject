package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(1, 1, 1);

    public static final double DEFAULT_MAX_POWER = 0.75;
    public static final double OVERDRIVE_POWER = 1.0;

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = 25; // Inches
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = 24; // Inches

    public static final double FINE_ADJUSTMENT_PERCENT = 0.2;

    public static final double MAX_SPEED = SwerveConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond); // SPEED_AT_12_VOLTS
                                                                                                  // desired top speed

    // 3/4 of a rotation angular velocity per second max
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
}