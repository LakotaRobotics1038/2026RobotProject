package frc.robot.utils;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.DriveConstants;

public final class RectangleUtils {
    private RectangleUtils() {
    }

    public static boolean drivingThroughRect(
            Rectangle2d[] rects,
            Translation2d robotPos,
            double vxMetersPerSecond,
            double vyMetersPerSecond) {
        for (Rectangle2d rect : rects) {
            if (drivingThroughRect(
                    rect,
                    robotPos,
                    vxMetersPerSecond,
                    vyMetersPerSecond)) {
                return true;
            }
        }
        return false;
    }

    public static boolean inRect(Rectangle2d[] rects, Translation2d robotPos) {
        for (Rectangle2d rect : rects) {
            if (inRect(rect, robotPos)) {
                return true;
            }
        }
        return false;
    }

    private static boolean drivingThroughRect(
            Rectangle2d rect,
            Translation2d robotPos,
            double vxMetersPerSecond,
            double vyMetersPerSecond) {
        Translation2d nearestRectPointToRobot = rect.nearest(robotPos);

        if (!inRect(robotPos, nearestRectPointToRobot)) {
            return false;
        }

        double speed = Math.hypot(vxMetersPerSecond, vyMetersPerSecond);
        if (speed < DriveConstants.BUMP_APPROACH_SPEED_THRESHOLD) {
            return false;
        }

        double dx = nearestRectPointToRobot.getX() - robotPos.getX();
        double dy = nearestRectPointToRobot.getY() - robotPos.getY();

        return vxMetersPerSecond * dx + vyMetersPerSecond * dy > 0;
    }

    private static boolean inRect(Rectangle2d rect, Translation2d robotPos) {
        return inRect(robotPos, rect.nearest(robotPos));
    }

    private static boolean inRect(
            Translation2d robotPos,
            Translation2d nearestRectPointToRobot) {
        return nearestRectPointToRobot.getDistance(robotPos) <= DriveConstants.ROBOT_SIZE_RADIUS;
    }
}
