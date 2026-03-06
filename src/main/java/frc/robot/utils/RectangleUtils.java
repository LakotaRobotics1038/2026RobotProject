package frc.robot.utils;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.DriveConstants;

/**
 * Utility methods for checking robot interaction with field rectangles.
 */
public final class RectangleUtils {
    private RectangleUtils() {
    }

    /**
     * Determines whether the robot is approaching any rectangle in the provided
     * set.
     *
     * <p>
     * The robot is considered to be "driving through" a rectangle when its
     * footprint intersects the rectangle (using {@link DriveConstants#ROBOT_SIZE_RADIUS})
     * and its translational velocity points toward the rectangle at or above
     * {@link DriveConstants#BUMP_APPROACH_SPEED_THRESHOLD}.
     *
     * @param rects             rectangles to check against
     * @param robotPos          robot translation in meters
     * @param vx robot x velocity in meters per second
     * @param vy robot y velocity in meters per second
     * @return true if the robot is driving into at least one rectangle
     */
    public static boolean drivingThroughRect(
            Rectangle2d[] rects,
            Translation2d robotPos,
            double vx,
            double vy) {
        for (Rectangle2d rect : rects) {
            if (drivingThroughRect(
                    rect,
                    robotPos,
                    vx,
                    vy)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Determines whether the robot's footprint overlaps any rectangle in the
     * provided set.
     *
     * @param rects    rectangles to check against
     * @param robotPos robot translation in meters
     * @return true if the robot is in at least one rectangle
     */
    public static boolean inRect(Rectangle2d[] rects, Translation2d robotPos) {
        for (Rectangle2d rect : rects) {
            if (isInRect(rect, robotPos)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Determines whether the robot is driving into a single rectangle.
     *
     * <p>
     * This requires all the following:
     * <ul>
     * <li>The robot is currently in the rectangle (accounting for robot radius)</li>
     * <li>The robot's speed is at least
     * {@link DriveConstants#BUMP_APPROACH_SPEED_THRESHOLD}</li>
     * <li>The velocity vector has a positive dot product toward the nearest point
     * on the rectangle</li>
     * </ul>
     *
     * @param rect              rectangle to check
     * @param robotPos          robot translation in meters
     * @param vx robot x velocity in meters per second
     * @param vy robot y velocity in meters per second
     * @return true if the robot is driving into the rectangle
     */
    private static boolean drivingThroughRect(
            Rectangle2d rect,
            Translation2d robotPos,
            double vx,
            double vy) {
        // Size of the velocity vector. If the robot is moving too slowly,
        // we don't consider it to be "approaching" the bump rectangle.
        double speed = Math.hypot(vx, vy);
        if (speed < DriveConstants.BUMP_APPROACH_SPEED_THRESHOLD) {
            return false;
        }

        Translation2d nearestRectPoint = rect.nearest(robotPos);
        if (!isWithinRobotRadius(robotPos, nearestRectPoint)) {
            return false;
        }

        // Vector from the robot to the nearest point on the rectangle
        double dx = nearestRectPoint.getX() - robotPos.getX();
        double dy = nearestRectPoint.getY() - robotPos.getY();

        // Use the dot product between velocity (vx,vy) and the displacement
        // vector (dx,dy). If the dot product is positive, the robot is moving in the
        // direction of the rectangle.
        return vx * dx + vy * dy > 0.0;
    }

    /**
     * Determines whether the robot's footprint overlaps a single rectangle.
     *
     * @param rect     rectangle to check
     * @param robotPos robot translation in meters
     * @return true if the robot is in the rectangle
     */
    private static boolean isInRect(Rectangle2d rect, Translation2d robotPos) {
        return isWithinRobotRadius(robotPos, rect.nearest(robotPos));
    }

    /**
     * Determines whether the distance from the robot position to the nearest
     * rectangle point is within the robot radius.
     *
     * @param robotPos         robot translation in meters
     * @param nearestRectPoint nearest point on the rectangle to the robot
     * @return true if the robot footprint overlaps the rectangle boundary
     */
    private static boolean isWithinRobotRadius(
            Translation2d robotPos,
            Translation2d nearestRectPoint) {
        return nearestRectPoint.getDistance(robotPos) <= DriveConstants.ROBOT_SIZE_RADIUS;
    }
}
