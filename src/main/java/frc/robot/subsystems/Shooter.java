package frc.robot.subsystems;

import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    private final ShooterModule nearShooter;
    private final ShooterModule farShooter;

    private Shooter() {
        ServoHub servoHub = new ServoHub(ShooterConstants.SERVO_HUB_CAN_ID);
        ServoHubConfig servoHubConfig = new ServoHubConfig();
        nearShooter = new ShooterModule(ShooterConstants.NEAR_SHOOTER_MODULE_CONSTANTS, servoHub, servoHubConfig);
        farShooter = new ShooterModule(ShooterConstants.FAR_SHOOTER_MODULE_CONSTANTS, servoHub, servoHubConfig);
        servoHub.configure(servoHubConfig, ResetMode.kResetSafeParameters);
        nearShooter.enableServo();
        farShooter.enableServo();
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    /**
     * Gets the shooter module that is closer to the hub.
     *
     * @return The near shooter module.
     */
    public ShooterModule getNearShooter() {
        return nearShooter;
    }

    /**
     * Gets the shooter module that is further from the hub.
     *
     * @return The far shooter module.
     */
    public ShooterModule getFarShooter() {
        return farShooter;
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
         * Creates and configures a shooter module.
         *
         * @param moduleConstants configuration for this shooter module. See
         *                        {@link ShooterConstants.ShooterModuleConstants}.
         * @param servoHub        ServoHub used to get and control the servo channel for
         *                        this module.
         * @param servoHubConfig  Configuration object whose channel settings are
         *                        updated for this module.
         */
        private ShooterModule(ShooterConstants.ShooterModuleConstants moduleConstants, ServoHub servoHub,
                ServoHubConfig servoHubConfig) {
            SparkFlexConfig baseConfig = new SparkFlexConfig();
            baseConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)
                    .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                    .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D).feedForward
                    .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

            SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
            leftMotorConfig.inverted(true).apply(baseConfig);
            leftMotor = new SparkFlex(moduleConstants.leftMotorCanId(), SparkLowLevel.MotorType.kBrushless);
            leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            SparkFlexConfig rightMotorConfig = new SparkFlexConfig();
            rightMotorConfig.apply(baseConfig).follow(leftMotor, true);
            rightMotor = new SparkFlex(moduleConstants.rightMotorCanId(), SparkLowLevel.MotorType.kBrushless);
            rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            controller = leftMotor.getClosedLoopController();
            encoder = leftMotor.getEncoder();

            translation = moduleConstants.translation();
            servoChannel = servoHub.getServoChannel(moduleConstants.servoChannelID());
            servoPulseRange = moduleConstants.servoPulseRange();

            ServoChannelConfig channelConfig;
            switch (moduleConstants.servoChannelID()) {
                case kChannelId0 -> channelConfig = servoHubConfig.channel0;
                case kChannelId1 -> channelConfig = servoHubConfig.channel1;
                case kChannelId2 -> channelConfig = servoHubConfig.channel2;
                case kChannelId3 -> channelConfig = servoHubConfig.channel3;
                case kChannelId4 -> channelConfig = servoHubConfig.channel4;
                case kChannelId5 -> channelConfig = servoHubConfig.channel5;
                default -> throw new IllegalArgumentException("Invalid servo channel ID"); // Will never get called
            }
            channelConfig.pulseRange(servoPulseRange).disableBehavior(BehaviorWhenDisabled.kSupplyPower);
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
         * Indicates whether the shooter is at the target RPM.
         *
         * @return Whether the shooter is at the target RPM.
         */
        public boolean isAtTargetRPM() {
            return Math.abs(getRPM() - getTargetRPM()) <= ShooterConstants.OPERATING_TOLERANCE;
        }

        /**
         * Gets the distance from this module to the hub.
         *
         * @param robotPose Robot pose in field coordinates.
         * @return Distance from this module to the hub.
         */
        public double getHubDistance(Pose2d robotPose) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            Translation2d hubPosition = alliance == Alliance.Blue ? FieldConstants.HUB_POSITION
                    : FlippingUtil.flipFieldPosition(FieldConstants.HUB_POSITION);
            Translation2d fieldPosition = robotPose.getTranslation()
                    .plus(translation.rotateBy(robotPose.getRotation()));
            return fieldPosition.getDistance(hubPosition);
        }

        /**
         * Calculates the angle from this module's location to the hub.
         *
         * @param robotPose Current robot pose in field coordinates.
         * @return Angle in radians from the module toward the hub.
         */
        public double getHubAngle(Pose2d robotPose) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            Translation2d moduleFieldPosition = robotPose.getTranslation()
                    .plus(translation.rotateBy(robotPose.getRotation()));
            Translation2d hubPosition = alliance == Alliance.Blue ? FieldConstants.HUB_POSITION
                    : FlippingUtil.flipFieldPosition(FieldConstants.HUB_POSITION);
            Translation2d toTargetFromModule = hubPosition.minus(moduleFieldPosition);
            return toTargetFromModule.getAngle().getRadians();
        }

        public void enableServo() {
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
                    ShooterConstants.SHOOTER_NO_RETRACTION_ANGLE,
                    ShooterConstants.SHOOTER_FULL_RETRACTION_ANGLE);
            // Sets the angle to a value between 0 and 1.
            double normalized = (ShooterConstants.SHOOTER_FULL_RETRACTION_ANGLE - clampedAngle)
                    / (ShooterConstants.SHOOTER_FULL_RETRACTION_ANGLE - ShooterConstants.SHOOTER_NO_RETRACTION_ANGLE);
            servoChannel.setPulseWidth(servoPulseRange.minPulse_us
                    + (int) (normalized * (servoPulseRange.maxPulse_us - servoPulseRange.minPulse_us)));
        }

        public void setPulseWidth(int pulseWidth_us) {
            servoChannel.setPulseWidth(pulseWidth_us);
        }
    }
}
