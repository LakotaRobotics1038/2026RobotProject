package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.json.simple.parser.ParseException;

public final class AutoConstants {
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 7;
    public static final double MAX_SPEED = SwerveConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double P_X_CONTROLLER = 8;
    public static final double I_X_CONTROLLER = 0.02;
    public static final double D_CONTROLLER = 0.0;
    public static final double P_THETA_CONTROLLER = 1.35;
    public static final double I_THETA_CONTROLLER = 0.0;
    public static final double D_THETA_CONTROLLER = 0.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final RobotConfig ROBOT_CONFIG;

    static {
        try {
            ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
