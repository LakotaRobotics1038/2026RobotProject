package frc.robot;

import java.util.function.Predicate;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class DriverJoystick extends XboxController1038 {
    // Subsystem Dependencies
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    // Commands
    // NONE

    // Instance Variables
    private double maxPower = DriveConstants.DEFAULT_MAX_POWER;
    private static final Rectangle2d[] BUMP_RECTANGLES = {
            FieldConstants.BLUE_LEFT_BUMP,
            FieldConstants.BLUE_RIGHT_BUMP,
            FieldConstants.RED_LEFT_BUMP,
            FieldConstants.RED_RIGHT_BUMP
    };
    private static final Rectangle2d[] TRENCH_RECTANGLES = {
            FieldConstants.BLUE_LEFT_TRENCH,
            FieldConstants.BLUE_RIGHT_TRENCH,
            FieldConstants.RED_LEFT_TRENCH,
            FieldConstants.RED_RIGHT_TRENCH
    };

    // Previous Status
    private double prevSideways = 0;
    private double prevForward = 0;
    private double prevRotate = 0;

    // Limiters
    SlewRateLimiter forwardLimiter = new SlewRateLimiter(2.0);
    SlewRateLimiter sidewaysLimiter = new SlewRateLimiter(2.0);
    SlewRateLimiter rotateLimiter = new SlewRateLimiter(2.0);

    LinearFilter forwardFilter = LinearFilter.movingAverage(5);
    LinearFilter sidewaysFilter = LinearFilter.movingAverage(5);

    private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

    // Singleton Setup
    private static DriverJoystick instance;

    public static DriverJoystick getInstance() {
        if (instance == null) {
            System.out.println("Creating a new Driver Xbox Controller");
            instance = new DriverJoystick();
        }
        return instance;
    }

    private DriverJoystick() {
        super(IOConstants.DRIVER_CONTROLLER_PORT);

        driveTrain.setDefaultCommand(this.driveTrain.applyRequest(() -> {
            if (maxPower != DriveConstants.OVERDRIVE_POWER) {
                maxPower = anyRectMatches(BUMP_RECTANGLES, this::drivingThroughRect)
                        ? DriveConstants.BUMP_SLOWDOWN_POWER
                        : DriveConstants.DEFAULT_MAX_POWER;
            }

            if (anyRectMatches(TRENCH_RECTANGLES, this::inRect)) {
                retractHoods();
            }

            double sideways = this.getSidewaysValue();
            double forward = this.getForwardValue();
            double rotate = this.getRotateValue();

            return driveTrain.drive(forward, -sideways, -rotate, true);
        }));

        this.driveTrain.registerTelemetry(logger::telemeterize);

        // Re-orient robot to the field
        this.start().whileTrue(new InstantCommand(driveTrain::seedFieldCentric, driveTrain));

        new Trigger(() -> this.getPOV().equals(PovPositions.Up))
                .whileTrue(this.driveTrain
                        .applyRequest(() -> driveTrain.drive(DriveConstants.FINE_ADJUSTMENT_PERCENT, 0, 0, false)));

        new Trigger(() -> this.getPOV().equals(PovPositions.Down))
                .whileTrue(this.driveTrain
                        .applyRequest(() -> driveTrain.drive(-DriveConstants.FINE_ADJUSTMENT_PERCENT, 0, 0, false)));

        new Trigger(() -> this.getPOV().equals(PovPositions.Left))
                .whileTrue(this.driveTrain
                        .applyRequest(() -> driveTrain.drive(0, DriveConstants.FINE_ADJUSTMENT_PERCENT, 0, false)));

        new Trigger(() -> this.getPOV().equals(PovPositions.Right))
                .whileTrue(this.driveTrain
                        .applyRequest(() -> driveTrain.drive(0, -DriveConstants.FINE_ADJUSTMENT_PERCENT, 0, false)));

        this.rightBumper()
                .onTrue(new InstantCommand(() -> this.maxPower = DriveConstants.OVERDRIVE_POWER))
                .onFalse(new InstantCommand(() -> this.maxPower = DriveConstants.DEFAULT_MAX_POWER));

        this.x().whileTrue(this.driveTrain.setX());
    }

    /**
     * Gets the value of the left X axis, filters it, and applies an acceleration
     * limit
     *
     * @return sideways value
     */
    private double getSidewaysValue() {
        double x = this.getLeftX() * maxPower;

        double sideways = limitRate(x, prevSideways, sidewaysLimiter);
        prevSideways = sideways;

        return sideways;
    }

    /**
     * Gets the value of the left Y axis, filters it, and applies an acceleration
     * limit
     *
     * @return forward value
     */
    private double getForwardValue() {
        double y = this.getLeftY() * maxPower;

        double forward = limitRate(y, prevForward, forwardLimiter);
        prevForward = forward;

        return forward;
    }

    /**
     * Gets the value of the Right X axis and applies an acceleration
     * limit
     *
     * @return rotate value
     */
    private double getRotateValue() {
        double z = this.getRightX() * maxPower;

        double rotate = limitRate(z, prevRotate, rotateLimiter);
        prevRotate = rotate;

        return rotate;
    }

    /**
     *
     * @param value   Current desired value
     * @param prevVal Previously desired value
     * @param filter  SlewRateLimiter instance for calculation
     * @return desired value rate limited and adjusted for sign changes using
     *         {@link #signChange Sign Change Function}
     */
    private double limitRate(double value, double prevVal, SlewRateLimiter filter) {
        if (value == 0 || signChange(value, prevVal)) {
            filter.reset(0);
        }
        return filter.calculate(value);
    }

    /**
     * Determines if the two given values are opposite signs
     * (one positive one negative)
     *
     * @param a first value to check sign
     * @param b second value to check sign
     * @return are the provided values different signs
     */
    private boolean signChange(double a, double b) {
        return a > 0 && b < 0 || b > 0 && a < 0;
    }

    /**
     * Determines if the robot is currently driving through a given rectangle on the
     * field. Takes into account the direction of the robot's movement and only
     * returns true if the robot is moving towards the rectangle.
     *
     * @param rect the rectangle to check
     * @return true if the robot is driving through the rectangle, false otherwise
     */
    private boolean drivingThroughRect(Rectangle2d rect) {
        Translation2d robotPos = driveTrain.getState().Pose.getTranslation();
        Translation2d nearestRectPointToRobot = rect.nearest(robotPos);

        if (!inRect(robotPos, nearestRectPointToRobot)) {
            return false;
        }

        double vx = driveTrain.getState().Speeds.vxMetersPerSecond;
        double vy = driveTrain.getState().Speeds.vyMetersPerSecond;

        // Magnitude of the velocity vector. If the robot is moving too slowly
        // we don't consider it to be 'approaching' the bump rectangle.
        double speed = Math.sqrt(vx * vx + vy * vy);

        if (speed < DriveConstants.BUMP_APPROACH_SPEED_THRESHOLD) {
            return false;
        }

        // Vector from the robot to the nearest point on the rectangle
        double dx = nearestRectPointToRobot.getX() - robotPos.getX();
        double dy = nearestRectPointToRobot.getY() - robotPos.getY();

        // Use the dot product between velocity (vx,vy) and the displacement
        // vector (dx,dy). If the dot product is positive, the robot is moving in the
        // direction of the rectangle.
        return vx * dx + vy * dy > 0;
    }

    private void retractHoods() {
        shooter.getNearShooter().setAngle(ShooterConstants.SHOOTER_ANGLE_MIN_DEG);
        shooter.getFarShooter().setAngle(ShooterConstants.SHOOTER_ANGLE_MIN_DEG);
    }

    private boolean anyRectMatches(Rectangle2d[] rects, Predicate<Rectangle2d> predicate) {
        for (Rectangle2d rect : rects) {
            if (predicate.test(rect)) {
                return true;
            }
        }
        return false;
    }

    private boolean inRect(Rectangle2d rect) {
        Translation2d robotPos = driveTrain.getState().Pose.getTranslation();
        return inRect(robotPos, rect.nearest(robotPos));
    }

    private boolean inRect(Translation2d robotPos, Translation2d nearestRectPointToRobot) {
        return nearestRectPointToRobot.getDistance(robotPos) <= DriveConstants.ROBOT_SIZE_RADIUS;
    }
}
