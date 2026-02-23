package frc.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    private FieldConstants() {}

    private static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);
    private static final double HUB_LENGTH = Units.inchesToMeters(47);
    private static final double HUB_CENTER_X = HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH / 2;
    private static final double HUB_CENTER_Y = FlippingUtil.fieldSizeY / 2;
    private static final double HUB_CENTER_Z = Units.inchesToMeters(72);

    public static final Translation3d HUB_POSITION = new Translation3d(HUB_CENTER_X, HUB_CENTER_Y, HUB_CENTER_Z);
}
