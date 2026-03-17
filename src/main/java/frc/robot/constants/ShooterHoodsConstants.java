package frc.robot.constants;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;

public final class ShooterHoodsConstants {
    public static final int SERVO_HUB_CAN_ID = 17;

    public static final ServoChannel.ChannelId NEAR_HOOD_CHANNEL_ID = ServoChannel.ChannelId.kChannelId1;
    public static final ServoChannelConfig.PulseRange NEAR_HOOD_PULSE_RANGE = new ServoChannelConfig.PulseRange(1000,
            1500, 2000);

    public static final ServoChannel.ChannelId FAR_HOOD_CHANNEL_ID = ServoChannel.ChannelId.kChannelId0;
    public static final ServoChannelConfig.PulseRange FAR_HOOD_PULSE_RANGE = new ServoChannelConfig.PulseRange(1000,
            1500, 2000);

    public static final double SHOOTER_NO_RETRACTION_ANGLE = 54.0;
    public static final double SHOOTER_FULL_RETRACTION_ANGLE = 73.0;

}
