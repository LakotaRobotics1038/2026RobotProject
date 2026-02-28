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
            0,
            0,
            new Translation2d(),
            ServoChannel.ChannelId.kChannelId0,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final ShooterModuleConstants FAR_SHOOTER_MODULE_CONSTANTS = new ShooterModuleConstants(
            0,
            0,
            new Translation2d(),
            ServoChannel.ChannelId.kChannelId0,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final int SERVO_HUB_CAN_ID = 0;

    public static final double POSITION_CONVERSION_FACTOR = 1;

    public static final double RPM_TOLERANCE = 25;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;

    public static final double SHOOTER_ANGLE_MIN_DEG = 55.0;
    public static final double SHOOTER_ANGLE_MAX_DEG = 70.0;

    public static class ShooterFormula {
        private final double slope;
        private final double yIntercept;
        private final double min;
        private final double max;
        private final double angle;

        private ShooterFormula(double angle, double slope, double yIntercept, double min, double max) {
            this.angle = angle;
            this.slope = slope;
            this.yIntercept = yIntercept;
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

        public double getRPM(double distance) {
            return slope * distance + yIntercept;
        }
    }

    /**
     * List of angles and their corresponding shooter formulas. The formula is used to calculate the RPM of the shooter
     * based on the distance to the target. The min and max values represent the range of that angle.
     */
    public static final List<ShooterFormula> SHOOTER_FORMULAS = List.of(
            new ShooterFormula(
                    55.0,
                    454.97,
                    2033.9,
                    1.7,
                    3.25),
            new ShooterFormula(
                    65.0,
                    448.4,
                    2056.9,
                    2.5,
                    5.1));
}
