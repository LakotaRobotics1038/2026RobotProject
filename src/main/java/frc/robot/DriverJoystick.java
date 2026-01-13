package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.DriveTrain;

public class DriverJoystick extends XboxController1038 {
    // Subsystem Dependencies
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    // Commands
    // NONE

    // Instance Variables
    private double maxPower = DriveConstants.DEFAULT_MAX_POWER;

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

        // Lock the wheels into an X formation
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

        double sideways = sidewaysFilter.calculate(x);
        sideways = limitRate(x, prevSideways, sidewaysLimiter);
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

        double forward = forwardFilter.calculate(y);
        forward = limitRate(y, prevForward, forwardLimiter);
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
        double z = this.getRightX() * 0.75;

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
}
