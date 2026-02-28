package frc.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    private static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);
    private static final double BUMP_WIDTH = Units.inchesToMeters(73);
    private static final double BUMP_LENGTH = Units.inchesToMeters(44.4);
    private static final double TRENCH_WIDTH = Units.inchesToMeters(65.65);
    private static final double LEFT_BUMP_DISTANCE = TRENCH_WIDTH;
    private static final double RIGHT_BUMP_DISTANCE = FlippingUtil.fieldSizeY - TRENCH_WIDTH - BUMP_WIDTH;
    private static final Rectangle2d BUMP = new Rectangle2d(
            new Translation2d(0, 0),
            new Translation2d(BUMP_WIDTH, BUMP_LENGTH));
    public static final Rectangle2d BLUE_LEFT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, LEFT_BUMP_DISTANCE),
            Rotation2d.kZero));
    public static final Rectangle2d BLUE_RIGHT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, RIGHT_BUMP_DISTANCE),
            Rotation2d.kZero));
    public static final Rectangle2d RED_LEFT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(FlippingUtil.fieldSizeX - HUB_EDGE_DISTANCE_FROM_DRIVER_STATION - BUMP_LENGTH, LEFT_BUMP_DISTANCE),
            Rotation2d.kZero));
    public static final Rectangle2d RED_RIGHT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(FlippingUtil.fieldSizeX - HUB_EDGE_DISTANCE_FROM_DRIVER_STATION - BUMP_LENGTH, RIGHT_BUMP_DISTANCE),
            Rotation2d.kZero));
}
