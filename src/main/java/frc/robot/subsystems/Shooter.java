package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    private final NearShooter nearShooter;
    private final FarShooter farShooter;

    private final ServoHub servoHub;

    private Shooter() {
        servoHub = new ServoHub(ShooterConstants.SERVO_HUB_CAN_ID);
        ServoHubConfig config = new ServoHubConfig();
        nearShooter = new NearShooter(config);
        farShooter = new FarShooter(config);
        servoHub.configure(config, ResetMode.kResetSafeParameters);
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private class NearShooter extends ShooterTemplate {
        private NearShooter(ServoHubConfig config) {
            super(ShooterConstants.NEAR_LEFT_MOTOR_CAN_ID, ShooterConstants.NEAR_RIGHT_MOTOR_CAN_ID, ShooterConstants.NEAR_TRANSLATION, ShooterConstants.NEAR_SERVO_CHANNEL);
            config.apply(ShooterConstants.NEAR_SERVO_CHANNEL, new ServoChannelConfig(ShooterConstants.NEAR_SERVO_CHANNEL).pulseRange(ShooterConstants.NEAR_SERVO_PULSE_RANGE));
        }
    }

    private class FarShooter extends ShooterTemplate {
        private FarShooter(ServoHubConfig config) {
            super(ShooterConstants.FAR_LEFT_MOTOR_CAN_ID, ShooterConstants.FAR_RIGHT_MOTOR_CAN_ID, ShooterConstants.FAR_TRANSLATION, ShooterConstants.FAR_SERVO_CHANNEL);
            config.apply(ShooterConstants.FAR_SERVO_CHANNEL, new ServoChannelConfig(ShooterConstants.FAR_SERVO_CHANNEL).pulseRange(ShooterConstants.FAR_SERVO_PULSE_RANGE));
        }
    }

    /**
     * Base shooter template.
     */
    private abstract class ShooterTemplate {
        private final SparkFlex leftMotor;
        private final SparkFlex rightMotor;
        private final SparkClosedLoopController controller;
        private final RelativeEncoder encoder;
        private final Translation3d translation;
        private final ChannelId servoChannel;

        /**
         * Creates a shooter with the specified motor controller CAN IDs.
         *
         * @param leftMotorCanId  CAN ID of the left shooter motor controller.
         * @param rightMotorCanId CAN ID of the right shooter motor controller.
         */
        protected ShooterTemplate(int leftMotorCanId, int rightMotorCanId, Translation3d translation, ChannelId servoChannel) {
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
            this.servoChannel = servoChannel;
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

        public ServoChannel getServoChannel() {
            return servoHub.getServoChannel(servoChannel);
        }
    }
}
