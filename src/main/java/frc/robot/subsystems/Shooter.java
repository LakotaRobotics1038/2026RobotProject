package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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
        nearShooter = new ShooterModule(ShooterConstants.NEAR_SHOOTER_MODULE_OPTIONS, servoHub, servoHubConfig);
        farShooter = new ShooterModule(ShooterConstants.FAR_SHOOTER_MODULE_OPTIONS, servoHub, servoHubConfig);
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
        private final Translation2d translation;
        private final ServoChannel servoChannel;
        private final ServoChannelConfig.PulseRange servoPulseRange;

        /**
         * Creates a shooter with the specified motor controller CAN IDs.
         *
         * @param leftMotorCanId  CAN ID of the left shooter motor controller.
         * @param rightMotorCanId CAN ID of the right shooter motor controller.
         */
        private ShooterModule(ShooterConstants.ShooterModuleOptions options, ServoHub servoHub,
                ServoHubConfig servoHubConfig) {
            SparkFlexConfig baseConfig = new SparkFlexConfig();
            baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                    .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
                    .allowedClosedLoopError(ShooterConstants.RPM_TOLERANCE, ClosedLoopSlot.kSlot0)
                    .outputRange(NeoMotorConstants.MIN_POWER, NeoMotorConstants.MAX_POWER).feedForward
                    .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

            SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
            leftMotorConfig.apply(baseConfig);
            leftMotor = new SparkFlex(options.leftMotorCanId(), SparkLowLevel.MotorType.kBrushless);
            leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            SparkFlexConfig rightMotorConfig = new SparkFlexConfig();
            rightMotorConfig.apply(baseConfig).follow(leftMotor, true);
            rightMotor = new SparkFlex(options.rightMotorCanId(), SparkLowLevel.MotorType.kBrushless);
            rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            controller = leftMotor.getClosedLoopController();
            encoder = leftMotor.getEncoder();

            translation = options.translation();
            servoChannel = servoHub.getServoChannel(options.servoChannelID());
            servoPulseRange = options.servoPulseRange();

            switch (options.servoChannelID()) {
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

        /**
         * Returns the target shooter speed.
         *
         * @return Current target speed in RPM.
         */
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

        /**
         * Gets the translation of the shooter module relative to the center of the
         * robot.
         *
         * @return The translation of the shooter module.
         */
        public Translation2d getTranslation() {
            return translation;
        }

        /**
         * Sets the servo angle by converting the given degrees to pulse width.
         *
         * @param angle Angle in degrees.
         */
        public void setAngle(double angle) {
            double clampedAngle = MathUtil.clamp(
                    angle,
                    ShooterConstants.SHOOTER_ANGLE_MIN_DEG,
                    ShooterConstants.SHOOTER_ANGLE_MAX_DEG);
            double normalized = (clampedAngle - ShooterConstants.SHOOTER_ANGLE_MIN_DEG)
                    / (ShooterConstants.SHOOTER_ANGLE_MAX_DEG - ShooterConstants.SHOOTER_ANGLE_MIN_DEG);
            servoChannel.setPulseWidth(servoPulseRange.minPulse_us
                    + (int) (normalized * (servoPulseRange.maxPulse_us - servoPulseRange.minPulse_us)));
        }

        /**
         * Sets the shooter to a certain speed given the distance to the hub. Assumes
         * that it is already aligned.
         *
         * @param hubDistance The distance from the shooter module to the hub.
         */
        public void autoShoot(double hubDistance) {
            for (Map.Entry<Double, ShooterConstants.ShooterFormula> entry : ShooterConstants.ANGLE_MAP.entrySet()) {
                ShooterConstants.ShooterFormula formula = entry.getValue();
                if (formula.getMin() <= hubDistance && formula.getMax() >= hubDistance) {
                    start(formula.getRPM(hubDistance));
                    break;
                }
            }
        }
    }
}
