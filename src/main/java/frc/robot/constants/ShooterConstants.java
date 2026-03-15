package frc.robot.constants;

import java.util.List;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
    public record ShooterModuleConstants(
            int leftMotorCanId,
            int rightMotorCanId,
            Translation2d translation,
            ServoChannel.ChannelId servoChannelID,
            ServoChannelConfig.PulseRange servoPulseRange) {
    }

    public static final ShooterModuleConstants NEAR_SHOOTER_MODULE_CONSTANTS = new ShooterModuleConstants(
            5,
            6,
            new Translation2d(Units.inchesToMeters(-13.5), Units.inchesToMeters(-7.826)),
            ServoChannel.ChannelId.kChannelId1,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final ShooterModuleConstants FAR_SHOOTER_MODULE_CONSTANTS = new ShooterModuleConstants(
            12,
            13,
            new Translation2d(Units.inchesToMeters(-13.5), Units.inchesToMeters(8.635)),
            ServoChannel.ChannelId.kChannelId0,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final int SERVO_HUB_CAN_ID = 17;

    public static final double OPERATING_TOLERANCE = 75;

    public static final double NEAR_SHOOTER_PERCENTAGE = 0.95;

    public static final double P = 0.000175;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.BATTERY_VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;

    public static final double SHOOTER_ANGLE_MIN_DEG = 54.0;
    public static final double SHOOTER_ANGLE_MAX_DEG = 73.0;
    public static final double SHOOTER_DIRECTION_FROM_FORWARD_RAD = -Math.PI / 2.0;
    public static final double MANUAL_SHOOTER_ANGLE_DEG = 59;
    public static final double MANUAL_SHOOTER_RPM = 2900.0;
    public static final double MANUAL_SHOOTER_RPM_STEP = 50.0;
    public static final double MANUAL_SHOOTER_MIN_RPM = 2000.0;
    // Make sure there's a 0 at the end so manual mode goes by 10s
    public static final double MANUAL_SHOOTER_MAX_RPM = (int) (NeoMotorConstants.VORTEX_FREE_SPEED_RPM / 10) * 10;

    /**
     * List of angles and their corresponding shooter formulas. The formula is used
     * to calculate the RPM of the shooter
     * based on the distance to the target. The min and max values represent the
     * range of that angle.
     */
    public static final List<ShooterFormula> SHOOTER_FORMULAS = List.of(
            new ShooterFormula(
                    59,
                    397.69,
                    1699.3,
                    1.524,
                    2.286),
            new ShooterFormula(
                    68,
                    393.7,
                    2150,
                    2.286,
                    10.0));

    public static final class ShooterFormula {
        private final double shooterSlope;
        private final double shooterYIntercept;
        private final double min;
        private final double max;
        private final double angle;

        private ShooterFormula(
                double angle,
                double shooterSlope,
                double shooterYIntercept,
                double min,
                double max) {
            this.angle = angle;
            this.shooterSlope = shooterSlope;
            this.shooterYIntercept = shooterYIntercept;
            this.min = min;
            this.max = max;
        }

        public double getAngle() {
            return angle;
        }

        public double getMin() {
            return min;
        }

        public double getMax() {
            return max;
        }

        public double getShooterRPM(double distance) {
            return shooterSlope * distance + shooterYIntercept;
        }
    }
}
