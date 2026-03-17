package frc.robot.subsystems;

import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        nearShooter = new ShooterModule(ShooterConstants.NEAR_SHOOTER_MODULE_CONSTANTS);
        farShooter = new ShooterModule(ShooterConstants.FAR_SHOOTER_MODULE_CONSTANTS);
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
     * Computes a virtual hub position offset by the robot's velocity to compensate
     * for lead shots while moving.
     *
     * @param robotPose           Current robot pose in field coordinates.
     * @param robotRelativeSpeeds Robot-relative chassis speeds.
     * @return Adjusted hub position that accounts for robot movement during time of flight.
     */
    public static Translation2d getVirtualHubPosition(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hubPosition = alliance == Alliance.Blue ? FieldConstants.HUB_POSITION
                : FlippingUtil.flipFieldPosition(FieldConstants.HUB_POSITION);

        // Convert robot-relative velocity to field-relative
        Translation2d fieldVelocity = new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond)
                .rotateBy(robotPose.getRotation());

        double distance = robotPose.getTranslation().getDistance(hubPosition);
        double tof = distance / ShooterConstants.SHOT_SPEED;

        return hubPosition.minus(new Translation2d(
                fieldVelocity.getX() * tof,
                fieldVelocity.getY() * tof));
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

        /**
         * Creates and configures a shooter module.
         *
         * @param moduleConstants configuration for this shooter module. See
         *                        {@link ShooterConstants.ShooterModuleConstants}.
         */
        private ShooterModule(ShooterConstants.ShooterModuleConstants moduleConstants) {
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
            return getHubDistance(robotPose, hubPosition);
        }

        /**
         * Gets the distance from this module to a custom target position.
         *
         * @param robotPose      Robot pose in field coordinates.
         * @param targetPosition Target position in field coordinates.
         * @return Distance from this module to the target.
         */
        public double getHubDistance(Pose2d robotPose, Translation2d targetPosition) {
            Translation2d fieldPosition = robotPose.getTranslation()
                    .plus(translation.rotateBy(robotPose.getRotation()));
            return fieldPosition.getDistance(targetPosition);
        }

        /**
         * Calculates the angle from this module's location to the hub.
         *
         * @param robotPose Current robot pose in field coordinates.
         * @return Angle in radians from the module toward the hub.
         */
        public double getHubAngle(Pose2d robotPose) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            Translation2d hubPosition = alliance == Alliance.Blue ? FieldConstants.HUB_POSITION
                    : FlippingUtil.flipFieldPosition(FieldConstants.HUB_POSITION);
            return getHubAngle(robotPose, hubPosition);
        }

        /**
         * Calculates the angle from this module's location to a custom target position.
         *
         * @param robotPose      Current robot pose in field coordinates.
         * @param targetPosition Target position in field coordinates.
         * @return Angle in radians from the module toward the target.
         */
        public double getHubAngle(Pose2d robotPose, Translation2d targetPosition) {
            Translation2d moduleFieldPosition = robotPose.getTranslation()
                    .plus(translation.rotateBy(robotPose.getRotation()));
            Translation2d toTargetFromModule = targetPosition.minus(moduleFieldPosition);
            return toTargetFromModule.getAngle().getRadians();
        }

    }
}
