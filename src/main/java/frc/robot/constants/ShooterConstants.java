package frc.robot.constants;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {

    public static final int SERVO_HUB_CAN_ID = 0;

    public static final int NEAR_LEFT_MOTOR_CAN_ID = 0;
    public static final int NEAR_RIGHT_MOTOR_CAN_ID = 0;
    public static final Translation2d NEAR_TRANSLATION = new Translation2d();
    public static final ServoChannel.ChannelId NEAR_SERVO_CHANNEL = ServoChannel.ChannelId.kChannelId0;
    public static final ServoChannelConfig.PulseRange NEAR_SERVO_PULSE_RANGE = new ServoChannelConfig.PulseRange(1000,
            1500, 2000);

    public static final int FAR_LEFT_MOTOR_CAN_ID = 0;
    public static final int FAR_RIGHT_MOTOR_CAN_ID = 0;
    public static final Translation2d FAR_TRANSLATION = new Translation2d();
    public static final ServoChannel.ChannelId FAR_SERVO_CHANNEL = ServoChannel.ChannelId.kChannelId1;
    public static final ServoChannelConfig.PulseRange FAR_SERVO_PULSE_RANGE = new ServoChannelConfig.PulseRange(1500,
            2000, 2500);

    public static final double RPM_TOLERANCE = 25.0;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;

    public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2);
    public static final double SHOOTER_ANGLE_MIN_DEG = 55.0;
    public static final double SHOOTER_ANGLE_MAX_DEG = 70.0;
}
