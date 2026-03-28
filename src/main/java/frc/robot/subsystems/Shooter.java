package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    private final SparkMax leftTop = new SparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT_TOP_CAN_ID,
            SparkLowLevel.MotorType.kBrushless);
    private final SparkMax leftBottom = new SparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT_BOTTOM_CAN_ID,
            SparkLowLevel.MotorType.kBrushless);
    private final SparkMax rightTop = new SparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT_TOP_CAN_ID,
            SparkLowLevel.MotorType.kBrushless);
    private final SparkMax rightBottom = new SparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT_BOTTOM_CAN_ID,
            SparkLowLevel.MotorType.kBrushless);

    private final SparkClosedLoopController controller = leftTop.getClosedLoopController();
    private final RelativeEncoder encoder = leftTop.getEncoder();

    private Shooter() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.idleMode(SparkBaseConfig.IdleMode.kCoast).inverted(true)
                .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D).feedForward
                .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

        leftTop.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        leftFollowerConfig.apply(baseConfig).follow(leftTop);
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
        rightFollowerConfig.apply(baseConfig).follow(leftTop, true);

        leftBottom.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightTop.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBottom.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        leftTop.stopMotor();
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
