package frc.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    private FieldConstants() {
    }

    private static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);
    private static final double BUMP_LENGTH = Units.inchesToMeters(44.4);
    private static final double BUMP_WIDTH = Units.inchesToMeters(73);
    private static final double TRENCH_WIDTH = Units.inchesToMeters(65.65);
    private static final double LEFT_BUMP_DISTANCE = TRENCH_WIDTH;
    private static final double RIGHT_BUMP_DISTANCE = FlippingUtil.fieldSizeY - TRENCH_WIDTH - BUMP_WIDTH;
    public static final Rectangle2d LEFT_BUMP = new Rectangle2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, LEFT_BUMP_DISTANCE),
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + BUMP_WIDTH, LEFT_BUMP_DISTANCE + BUMP_LENGTH));
    public static final Rectangle2d RIGHT_BUMP = new Rectangle2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, RIGHT_BUMP_DISTANCE),
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + BUMP_WIDTH, RIGHT_BUMP_DISTANCE + BUMP_LENGTH));
}
