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

    private final SparkFlex motor1 = new SparkFlex(ShooterConstants.SHOOTER_MOTOR_1_CAN_ID,
            SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex motor2 = new SparkFlex(ShooterConstants.SHOOTER_MOTOR_2_CAN_ID,
            SparkLowLevel.MotorType.kBrushless);

    private final SparkClosedLoopController controller = motor1.getClosedLoopController();
    private final RelativeEncoder encoder = motor1.getEncoder();

    private Shooter() {
        SparkFlexConfig baseConfig = new SparkFlexConfig();
        baseConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D).feedForward
                .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

        motor1.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.apply(baseConfig).follow(motor1);

        motor2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
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
        motor1.stopMotor();
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
        Translation2d targetPosition = FieldConstants.targetPosition();
        Translation2d fieldPosition = robotPose.getTranslation()
                .plus(ShooterConstants.SHOOTER_BARREL_CENTER.rotateBy(robotPose.getRotation()));
        return fieldPosition.getDistance(targetPosition);
    }

    /**
     * Calculates the angle from this module's location to the hub.
     *
     * @param robotPose Current robot pose in field coordinates.
     * @return Angle in radians from the module toward the hub.
     */
    public double getTargetAngle(Pose2d robotPose) {
        Translation2d targetPosition = FieldConstants.targetPosition();
        Translation2d moduleFieldPosition = robotPose.getTranslation()
                .plus(ShooterConstants.SHOOTER_BARREL_CENTER.rotateBy(robotPose.getRotation()));
        Translation2d toTargetFromModule = targetPosition.minus(moduleFieldPosition);
        return toTargetFromModule.getAngle().getRadians();
    }
}
