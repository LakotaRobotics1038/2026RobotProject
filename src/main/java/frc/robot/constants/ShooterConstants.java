package frc.robot.constants;

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
            new Translation2d(Units.inchesToMeters(13.5), Units.inchesToMeters(7.826)),
            ServoChannel.ChannelId.kChannelId1,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final ShooterModuleConstants FAR_SHOOTER_MODULE_CONSTANTS = new ShooterModuleConstants(
            12,
            13,
            new Translation2d(Units.inchesToMeters(13.5), Units.inchesToMeters(-8.635)),
            ServoChannel.ChannelId.kChannelId0,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final int SERVO_HUB_CAN_ID = 17;

    public static final double RPM_TOLERANCE = 25;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.BATTERY_VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;

    public static final double SHOOTER_ANGLE_MIN_DEG = 55.0;
    public static final double SHOOTER_ANGLE_MAX_DEG = 70.0;
    public static final double SHOOTER_DIRECTION_FROM_FORWARD_RAD = -Math.PI / 2.0;
    public static final double MANUAL_SHOOTER_ANGLE_DEG = SHOOTER_ANGLE_MIN_DEG;
    public static final double MANUAL_SHOOTER_RPM = 3200.0;
    public static final double MANUAL_SHOOTER_RPM_STEP = 50.0;
    public static final double MANUAL_SHOOTER_MIN_RPM = 0.0;
    public static final double MANUAL_SHOOTER_MAX_RPM = NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
}
