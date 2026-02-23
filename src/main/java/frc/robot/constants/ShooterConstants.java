package frc.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
    private ShooterConstants() {
    }

    public static final int SERVO_HUB_CAN_ID = 0;

    public static final int NEAR_LEFT_MOTOR_CAN_ID = 0;
    public static final int NEAR_RIGHT_MOTOR_CAN_ID = 0;
    public static final Translation3d NEAR_TRANSLATION = new Translation3d();
    public static final ServoChannel.ChannelId NEAR_SERVO_CHANNEL = ServoChannel.ChannelId.kChannelId0;
    public static final ServoChannelConfig.PulseRange NEAR_SERVO_PULSE_RANGE = new ServoChannelConfig.PulseRange(1000, 1500, 2000);

    public static final int FAR_LEFT_MOTOR_CAN_ID = 0;
    public static final int FAR_RIGHT_MOTOR_CAN_ID = 0;
    public static final Translation3d FAR_TRANSLATION = new Translation3d();
    public static final ServoChannel.ChannelId FAR_SERVO_CHANNEL = ServoChannel.ChannelId.kChannelId1;
    public static final ServoChannelConfig.PulseRange FAR_SERVO_PULSE_RANGE = new ServoChannelConfig.PulseRange(1500, 2000, 2500);

    public static final double RPM_TOLERANCE = 25.0;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;

    public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2);
    public static final double SHOOTER_ANGLE_MAX_DEG = 65.0;
    public static final double SHOOTER_ANGLE_MIN_DEG = 55.0;

    public static final double FUEL_DIAMETER = 0.15;
    public static final double FUEL_WEIGHT_MIN = Units.lbsToKilograms(0.448);
    public static final double FUEL_WEIGHT_MAX_LB = Units.lbsToKilograms(0.5);

    private static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);
    private static final double HUB_LENGTH = Units.inchesToMeters(47);
    private static final double HUB_CENTER_X = HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH / 2;
    private static final double HUB_CENTER_Y = FlippingUtil.fieldSizeY / 2;
    private static final double HUB_CENTER_Z = Units.inchesToMeters(72);

    public static final Translation3d HUB_POSITION = new Translation3d(HUB_CENTER_X, HUB_CENTER_Y, HUB_CENTER_Z);

    public enum ShooterAngle {
        LOW(55, -1),
        HIGH(65, -1);
        private final double angle;
        private final int pulseWidth;
        ShooterAngle(double angle, int pulseWidth) {
            this.angle = angle;
            this.pulseWidth = pulseWidth;
        }

        public double getAngle() {
            return angle;
        }

        public int getPulseWidth() {
            return pulseWidth;
        }
    }
}
