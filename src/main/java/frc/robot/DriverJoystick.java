package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AdjustHoodCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ObstacleAlignCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterHood;
import frc.robot.utils.RectangleUtils;

public class DriverJoystick extends XboxController1038 {
    // Subsystem Dependencies
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final ShooterHood shooterHood = ShooterHood.getInstance();

    // Commands
    // NONE

    // Instance Variables
    private double maxPower = DriveConstants.DEFAULT_MAX_POWER;

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
            if (!Dashboard.MANUAL_MODE_ENABLED.get()) {
                SwerveDrivetrain.SwerveDriveState state = driveTrain.getState();
                Translation2d robotPos = state.Pose.getTranslation();
                double vx = state.Speeds.vxMetersPerSecond;
                double vy = state.Speeds.vyMetersPerSecond;
                maxPower = RectangleUtils.drivingThroughRect(FieldConstants.BUMP_RECTANGLES, robotPos, vx, vy)
                        ? DriveConstants.BUMP_SLOWDOWN_POWER
                        : DriveConstants.DEFAULT_MAX_POWER;
            }

            double sideways = this.getSidewaysValue();
            double forward = this.getForwardValue();
            double rotate = this.getRotateValue();

            return driveTrain.drive(forward, -sideways, -rotate, true);
        }));

        shooterHood.setDefaultCommand(new AdjustHoodCommand());

        this.driveTrain.registerTelemetry(logger::telemeterize);

        // Re-orient robot to the field
        this.start().whileTrue(new InstantCommand(driveTrain::seedFieldCentric, driveTrain));

        this.x().whileTrue(this.driveTrain.setX());

        this.leftBumper().whileTrue(new ObstacleAlignCommand(this::getForwardValue, this::getSidewaysValue));
        this.leftTrigger().and(() -> !Dashboard.MANUAL_MODE_ENABLED.get()).whileTrue(new AlignCommand(
                this::getForwardValue,
                this::getSidewaysValue));

        new Trigger(Dashboard.HUB_ALIGNING::get)
                .onTrue(new InstantCommand(() -> setRumble(AlignCommand.HUB_ALIGNMENT_RUMBLE_INTENSITY)))
                .onFalse(new InstantCommand(() -> setRumble(0)));
    }

    /**
     * Gets the value of the left X axis, filters it, and applies an acceleration
     * limit
     *
     * @return sideways value
     */
    private double getSidewaysValue() {
        double sidewaysPower = Math.pow(Math.abs(this.getLeftX()), DriveConstants.JOYSTICK_EXPONENT);
        sidewaysPower = Math.copySign(sidewaysPower, this.getLeftX());
        double x = sidewaysPower * maxPower;
        return x;
    }

    /**
     * Gets the value of the left Y axis, filters it, and applies an acceleration
     * limit
     *
     * @return forward value
     */
    private double getForwardValue() {
        double forwardPower = Math.pow(Math.abs(this.getLeftY()), DriveConstants.JOYSTICK_EXPONENT);
        forwardPower = Math.copySign(forwardPower, this.getLeftY());
        double y = forwardPower * maxPower;
        return y;
    }

    /**
     * Gets the value of the Right X axis and applies an acceleration
     * limit
     *
     * @return rotate value
     */
    private double getRotateValue() {
        double rotatePower = Math.pow(Math.abs(this.getRightX()), DriveConstants.JOYSTICK_EXPONENT);
        rotatePower = Math.copySign(rotatePower, this.getRightX());
        double z = rotatePower * maxPower;
        return z;
    }
}
