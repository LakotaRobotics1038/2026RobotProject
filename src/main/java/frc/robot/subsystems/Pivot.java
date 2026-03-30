package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.PivotConstants.PivotSetpoint;
import frc.robot.constants.NeoMotorConstants;

public class Pivot extends SubsystemBase {
    private final SparkMax motor = new SparkMax(PivotConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    private final AbsoluteEncoder pivotEncoder = motor.getAbsoluteEncoder();

    private final SparkClosedLoopController pivotController = motor.getClosedLoopController();

    private static Pivot instance = null;

    private Pivot() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.apply(baseConfig).inverted(true).idleMode(IdleMode.kBrake).closedLoop
                .outputRange(-PivotConstants.POWER, PivotConstants.POWER)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(PivotConstants.P, PivotConstants.I,
                        PivotConstants.D)
                .allowedClosedLoopError(PivotConstants.ALLOWED_ERROR_DEGREES,
                        ClosedLoopSlot.kSlot0);
        pivotConfig.absoluteEncoder.positionConversionFactor(PivotConstants.ENCODER_CONVERSION_FACTOR)
                .inverted(true);

        motor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Gets the instance of Acquisition. Instantiates Acquisition if null.
     */
    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    /**
     * Sets the pivot motor position.
     *
     * @param setpoint The setpoint for the pivot motor to go to.
     */
    public void setPivot(PivotSetpoint setpoint) {
        setPivotDegrees(setpoint.getDegrees());
    }

    public void setPivotDegrees(double degrees) {
        pivotController.setSetpoint(
                MathUtil.clamp(degrees, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE),
                ControlType.kPosition);
    }

    public void stopPivot() {
        motor.stopMotor();
    }

    /**
     * Gets if the pivot motor is at the setpoint.
     */
    public boolean pivotAtSetpoint() {
        return MathUtil.isNear(pivotController.getSetpoint(), getPivotPosition(),
                PivotConstants.OPERATING_TOLERANCE);
    }

    /**
     * Gets the position of the pivot encoder.
     */
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }
}
