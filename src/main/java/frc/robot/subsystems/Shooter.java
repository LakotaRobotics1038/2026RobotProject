package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    private final ShooterModule nearShooter = new ShooterModule(ShooterConstants.NEAR_SHOOTER_MODULE_CONSTANTS);
    private final ShooterModule farShooter = new ShooterModule(ShooterConstants.FAR_SHOOTER_MODULE_CONSTANTS);

    private Shooter() {
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
        private final SparkFlex motor;
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

            SparkFlexConfig motorConfig = new SparkFlexConfig();
            motorConfig.inverted(true).apply(baseConfig);
            motor = new SparkFlex(moduleConstants.motorCanId(), SparkLowLevel.MotorType.kBrushless);
            motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            controller = motor.getClosedLoopController();
            encoder = motor.getEncoder();

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
            motor.stopMotor();
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
            return MathUtil.isNear(getRPM(), getTargetRPM(), ShooterConstants.OPERATING_TOLERANCE);
        }

        /**
         * Gets the distance from this module to the hub.
         *
         * @param robotPose Robot pose in field coordinates.
         * @return Distance from this module to the hub.
         */
        public double getTargetDistance(Pose2d robotPose) {
            Translation2d targetPosition = FieldConstants.targetPosition(robotPose.getTranslation());
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
        public double getTargetAngle(Pose2d robotPose) {
            Translation2d targetPosition = FieldConstants.targetPosition(robotPose.getTranslation());
            Translation2d moduleFieldPosition = robotPose.getTranslation()
                    .plus(translation.rotateBy(robotPose.getRotation()));
            Translation2d toTargetFromModule = targetPosition.minus(moduleFieldPosition);
            return toTargetFromModule.getAngle().getRadians();
        }

    }
}
