package frc.robot.constants;

import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldConstants {

    private static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);
    private static final double HUB_LENGTH = Units.inchesToMeters(47);
    private static final double HUB_CENTER_X = HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH / 2;
    private static final double HUB_CENTER_Y = FlippingUtil.fieldSizeY / 2;
    private static final double HUB_LEFT_Y = HUB_CENTER_Y - HUB_LENGTH / 2;
    private static final double HUB_RIGHT_Y = HUB_CENTER_Y + HUB_LENGTH / 2;

    public static final Translation2d HUB_POSITION = new Translation2d(HUB_CENTER_X, HUB_CENTER_Y);

    private static final double BUMP_WIDTH = Units.inchesToMeters(73);
    private static final double BUMP_DEPTH = Units.inchesToMeters(44.4);
    private static final double TRENCH_WIDTH = Units.inchesToMeters(65.65);

    private static final double LEFT_TRENCH_Y_OFFSET = FlippingUtil.fieldSizeY - TRENCH_WIDTH;
    private static final double RIGHT_TRENCH_Y_OFFSET = 0;

    private static final double LEFT_BUMP_Y_OFFSET = LEFT_TRENCH_Y_OFFSET - BUMP_WIDTH;
    private static final double RIGHT_BUMP_Y_OFFSET = TRENCH_WIDTH;

    private static final double RED_SIDE_DISTANCE = FlippingUtil.fieldSizeX - HUB_EDGE_DISTANCE_FROM_DRIVER_STATION
            - BUMP_DEPTH;

    private static final Rectangle2d BUMP = new Rectangle2d(
            new Translation2d(0, 0),
            new Translation2d(BUMP_DEPTH, BUMP_WIDTH));
    private static final Rectangle2d BLUE_LEFT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, LEFT_BUMP_Y_OFFSET),
            Rotation2d.kZero));
    private static final Rectangle2d BLUE_RIGHT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, RIGHT_BUMP_Y_OFFSET),
            Rotation2d.kZero));
    private static final Rectangle2d RED_LEFT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(RED_SIDE_DISTANCE, LEFT_BUMP_Y_OFFSET),
            Rotation2d.kZero));
    private static final Rectangle2d RED_RIGHT_BUMP = BUMP.transformBy(new Transform2d(
            new Translation2d(RED_SIDE_DISTANCE, RIGHT_BUMP_Y_OFFSET),
            Rotation2d.kZero));
    public static final List<Rectangle2d> BUMP_RECTANGLES = List.of(
            BLUE_LEFT_BUMP,
            BLUE_RIGHT_BUMP,
            RED_LEFT_BUMP,
            RED_RIGHT_BUMP);

    private static final Rectangle2d TRENCH = new Rectangle2d(
            new Translation2d(0, 0),
            new Translation2d(BUMP_DEPTH, TRENCH_WIDTH));

    private static final Rectangle2d BLUE_LEFT_TRENCH = TRENCH.transformBy(new Transform2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, LEFT_TRENCH_Y_OFFSET),
            Rotation2d.kZero));
    private static final Rectangle2d BLUE_RIGHT_TRENCH = TRENCH.transformBy(new Transform2d(
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, RIGHT_TRENCH_Y_OFFSET),
            Rotation2d.kZero));
    private static final Rectangle2d RED_LEFT_TRENCH = TRENCH.transformBy(new Transform2d(
            new Translation2d(RED_SIDE_DISTANCE, LEFT_TRENCH_Y_OFFSET),
            Rotation2d.kZero));
    private static final Rectangle2d RED_RIGHT_TRENCH = TRENCH.transformBy(new Transform2d(
            new Translation2d(RED_SIDE_DISTANCE, RIGHT_TRENCH_Y_OFFSET),
            Rotation2d.kZero));
    public static final List<Rectangle2d> TRENCH_RECTANGLES = List.of(
            BLUE_LEFT_TRENCH,
            BLUE_RIGHT_TRENCH,
            RED_LEFT_TRENCH,
            RED_RIGHT_TRENCH);

    private static final Rectangle2d LEFT_ALLIANCE = new Rectangle2d(
            new Translation2d(0, 0),
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, HUB_LEFT_Y));
    private static final Rectangle2d RIGHT_ALLIANCE = new Rectangle2d(
            new Translation2d(0, HUB_RIGHT_Y),
            new Translation2d(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, FlippingUtil.fieldSizeY));

    public static Translation2d targetPosition(Translation2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (robotPose.getX() > HUB_EDGE_DISTANCE_FROM_DRIVER_STATION) {
            Rectangle2d leftAllianceBoundingBox;
            Rectangle2d rightAllianceBoundingBox;
            if (alliance == Alliance.Blue) {
                leftAllianceBoundingBox = LEFT_ALLIANCE;
                rightAllianceBoundingBox = RIGHT_ALLIANCE;
            } else {
                leftAllianceBoundingBox = LEFT_ALLIANCE.transformBy(
                        new Transform2d(FlippingUtil.fieldSizeX - LEFT_ALLIANCE.getXWidth(), 0, Rotation2d.kZero));
                rightAllianceBoundingBox = RIGHT_ALLIANCE.transformBy(
                        new Transform2d(FlippingUtil.fieldSizeX - RIGHT_ALLIANCE.getXWidth(), 0, Rotation2d.kZero));
            }
            Translation2d leftNear = leftAllianceBoundingBox.nearest(robotPose);
            Translation2d rightNear = rightAllianceBoundingBox.nearest(robotPose);

            return leftNear.getDistance(robotPose) <= rightNear.getDistance(robotPose) ? leftNear : rightNear;
        } else {
            return alliance == Alliance.Blue ? HUB_POSITION : FlippingUtil.flipFieldPosition(HUB_POSITION);
        }
    }
}
