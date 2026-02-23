package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    private final ShooterModule nearShooter;
    private final ShooterModule farShooter;

    private final ServoHub servoHub;

    private Shooter() {
        servoHub = new ServoHub(ShooterConstants.SERVO_HUB_CAN_ID);
        ServoHubConfig servoHubConfig = new ServoHubConfig();
        nearShooter = new ShooterModule(ShooterConstants.NEAR_LEFT_MOTOR_CAN_ID,
                ShooterConstants.NEAR_RIGHT_MOTOR_CAN_ID, ShooterConstants.NEAR_TRANSLATION, servoHub, servoHubConfig,
                ShooterConstants.NEAR_SERVO_CHANNEL, ShooterConstants.NEAR_SERVO_PULSE_RANGE);
        farShooter = new ShooterModule(ShooterConstants.FAR_LEFT_MOTOR_CAN_ID, ShooterConstants.FAR_RIGHT_MOTOR_CAN_ID,
                ShooterConstants.FAR_TRANSLATION, servoHub, servoHubConfig, ShooterConstants.FAR_SERVO_CHANNEL,
                ShooterConstants.FAR_SERVO_PULSE_RANGE);
        servoHub.configure(servoHubConfig, ResetMode.kResetSafeParameters);
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public ShooterModule getNearShooter() {
        return nearShooter;
    }

    public ShooterModule getFarShooter() {
        return farShooter;
    }

    public ServoHub getServoHub() {
        return servoHub;
    }

    /**
     * Base shooter module.
     */
    public static class ShooterModule {
        private final SparkFlex leftMotor;
        private final SparkFlex rightMotor;
        private final SparkClosedLoopController controller;
        private final RelativeEncoder encoder;
        private final Translation3d translation;
        private final ServoChannel servoChannel;
        private final ServoChannelConfig.PulseRange servoPulseRange;

        /**
         * Creates a shooter with the specified motor controller CAN IDs.
         *
         * @param leftMotorCanId  CAN ID of the left shooter motor controller.
         * @param rightMotorCanId CAN ID of the right shooter motor controller.
         */
        private ShooterModule(int leftMotorCanId, int rightMotorCanId, Translation3d translation, ServoHub servoHub,
                ServoHubConfig servoHubConfig, ServoChannel.ChannelId servoChannelID,
                ServoChannelConfig.PulseRange servoPulseRange) {
            SparkFlexConfig baseConfig = new SparkFlexConfig();
            baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                    .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
                    .allowedClosedLoopError(ShooterConstants.RPM_TOLERANCE, ClosedLoopSlot.kSlot0)
                    .outputRange(NeoMotorConstants.MIN_POWER, NeoMotorConstants.MAX_POWER).feedForward
                    .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

            SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
            leftMotorConfig.apply(baseConfig);
            leftMotor = new SparkFlex(leftMotorCanId, SparkLowLevel.MotorType.kBrushless);
            leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            SparkFlexConfig rightMotorConfig = new SparkFlexConfig();
            rightMotorConfig.apply(baseConfig).follow(leftMotor, true);
            rightMotor = new SparkFlex(rightMotorCanId, SparkLowLevel.MotorType.kBrushless);
            rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            controller = leftMotor.getClosedLoopController();
            encoder = leftMotor.getEncoder();

            this.translation = translation;
            this.servoChannel = servoHub.getServoChannel(servoChannelID);
            this.servoPulseRange = servoPulseRange;

            switch (servoChannelID) {
                case kChannelId0 -> servoHubConfig.channel0.pulseRange(servoPulseRange);
                case kChannelId1 -> servoHubConfig.channel1.pulseRange(servoPulseRange);
                case kChannelId2 -> servoHubConfig.channel2.pulseRange(servoPulseRange);
                case kChannelId3 -> servoHubConfig.channel3.pulseRange(servoPulseRange);
                case kChannelId4 -> servoHubConfig.channel4.pulseRange(servoPulseRange);
                case kChannelId5 -> servoHubConfig.channel5.pulseRange(servoPulseRange);
            }

            servoChannel.setEnabled(true);
            servoChannel.setPowered(true);
        }

        /**
         * Starts the shooter at an RPM.
         *
         * @param rpm Target shooter speed in RPM.
         */
        public void start(double rpm) {
            controller.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
        }

        /**
         * Stops the shooter.
         */
        public void stop() {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }

        /**
         * Returns the current shooter speed.
         *
         * @return Current shooter speed in RPM.
         */
        public double getRPM() {
            return encoder.getVelocity();
        }

        public double getTargetRPM() {
            return controller.getSetpoint();
        }

        /**
         * Indicates whether the shooter at its target RPM.
         *
         * @return Whether the shooter is at the target RPM.
         */
        public boolean isAtTargetRPM() {
            return controller.isAtSetpoint();
        }

        public Translation3d getTranslation() {
            return translation;
        }

        private int getPulseWidth(double angle) {
            return servoPulseRange.minPulse_us +
                    (int) (MathUtil.clamp(angle, ShooterConstants.SHOOTER_ANGLE_MIN_DEG, ShooterConstants.SHOOTER_ANGLE_MAX_DEG) /
                            (ShooterConstants.SHOOTER_ANGLE_MAX_DEG - ShooterConstants.SHOOTER_ANGLE_MIN_DEG) * (servoPulseRange.maxPulse_us - servoPulseRange.minPulse_us));
        }

        public void setAngle(double angle) {
            servoChannel.setPulseWidth(getPulseWidth(angle));
        }
    }
}
