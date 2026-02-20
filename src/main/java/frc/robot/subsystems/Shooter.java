package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterConstants;

/**
 * Base shooter subsystem.
 */
public abstract class Shooter extends SubsystemBase {
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;
    private final Translation3d translation;
    private double rpm;

    /**
     * Creates a shooter with the specified motor controller CAN IDs.
     *
     * @param leftMotorCanId  CAN ID of the left shooter motor controller.
     * @param rightMotorCanId CAN ID of the right shooter motor controller.
     */
    protected Shooter(int leftMotorCanId, int rightMotorCanId, Translation3d translation) {
        SparkFlexConfig baseConfig = new SparkFlexConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
                .allowedClosedLoopError(ShooterConstants.RPM_TOLERANCE, ClosedLoopSlot.kSlot0)
                .outputRange(NeoMotorConstants.MIN_POWER, NeoMotorConstants.MAX_POWER).feedForward
                .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

        SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
        leftMotorConfig.apply(baseConfig);
        leftMotor = new SparkFlex(leftMotorCanId, MotorType.kBrushless);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig rightMotorConfig = new SparkFlexConfig();
        rightMotorConfig.apply(baseConfig).follow(leftMotor, true);
        rightMotor = new SparkFlex(rightMotorCanId, MotorType.kBrushless);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = leftMotor.getClosedLoopController();
        encoder = leftMotor.getEncoder();

        this.translation = translation;
    }

    /**
     * Starts the shooter at an RPM.
     *
     * @param rpm Target shooter speed in RPM.
     */
    public void start(double rpm) {
        this.rpm = rpm;
        controller.setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Stops the shooter.
     */
    public void stop() {
        this.rpm = 0;
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
        return rpm;
    }

    /**
     * Indicates whether the shooter at its target RPM.
     *
     * @return Whether the shooter is at the target RPM.
     */
    public boolean isAtTargetRPM() {
        return controller.isAtSetpoint();
    }
}
