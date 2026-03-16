package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterHoodsConstants;

public class ShooterHoods extends SubsystemBase {
    private final ServoHub servoHub = new ServoHub(ShooterHoodsConstants.SERVO_HUB_CAN_ID);
    private static ShooterHoods instance;

    private final Hood nearHood;
    private final Hood farHood;

    private ShooterHoods() {
        ServoHubConfig servoHubConfig = new ServoHubConfig();
        nearHood = new Hood(
                ShooterHoodsConstants.NEAR_HOOD_CHANNEL_ID,
                ShooterHoodsConstants.NEAR_HOOD_PULSE_RANGE,
                servoHubConfig);
        farHood = new Hood(
                ShooterHoodsConstants.FAR_HOOD_CHANNEL_ID,
                ShooterHoodsConstants.FAR_HOOD_PULSE_RANGE,
                servoHubConfig);
        servoHub.configure(servoHubConfig, ResetMode.kResetSafeParameters);
        nearHood.enable();
        farHood.enable();
    }

    public static ShooterHoods getInstance() {
        if (instance == null) {
            instance = new ShooterHoods();
        }
        return instance;
    }

    public Hood getNearHood() {
        return nearHood;
    }

    public Hood getFarHood() {
        return farHood;
    }

    public class Hood {
        private final ServoChannel servoChannel;
        private final ServoChannelConfig.PulseRange pulseRange;

        private Hood(ChannelId channelId, ServoChannelConfig.PulseRange pulseRange, ServoHubConfig servoHubConfig) {
            this.pulseRange = pulseRange;
            this.servoChannel = servoHub.getServoChannel(channelId);

            ServoChannelConfig channelConfig;
            switch (channelId) {
                case kChannelId0 -> channelConfig = servoHubConfig.channel0;
                case kChannelId1 -> channelConfig = servoHubConfig.channel1;
                case kChannelId2 -> channelConfig = servoHubConfig.channel2;
                case kChannelId3 -> channelConfig = servoHubConfig.channel3;
                case kChannelId4 -> channelConfig = servoHubConfig.channel4;
                case kChannelId5 -> channelConfig = servoHubConfig.channel5;
                default -> throw new IllegalArgumentException("Invalid servo channel ID");
            }
            channelConfig.pulseRange(pulseRange).disableBehavior(BehaviorWhenDisabled.kSupplyPower);
        }

        public void enable() {
            servoChannel.setEnabled(true);
            servoChannel.setPowered(true);
        }

        /**
         * Sets the hood angle by converting the given degrees to pulse width.
         *
         * @param angle Angle in degrees.
         */
        public void setAngle(double angle) {
            double clampedAngle = MathUtil.clamp(
                    angle,
                    ShooterHoodsConstants.SHOOTER_NO_RETRACTION_ANGLE,
                    ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE);
            double normalized = (ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE - clampedAngle)
                    / (ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE
                            - ShooterHoodsConstants.SHOOTER_NO_RETRACTION_ANGLE);
            servoChannel.setPulseWidth(pulseRange.minPulse_us
                    + (int) (normalized * (pulseRange.maxPulse_us - pulseRange.minPulse_us)));
        }
    }
}
