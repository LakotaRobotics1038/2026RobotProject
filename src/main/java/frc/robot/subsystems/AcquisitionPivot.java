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
import frc.robot.constants.AcquisitionPivotConstants;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;
import frc.robot.constants.NeoMotorConstants;

public class AcquisitionPivot extends SubsystemBase {
    private final SparkMax motor = new SparkMax(AcquisitionPivotConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    private final AbsoluteEncoder pivotEncoder = motor.getAbsoluteEncoder();

    private final SparkClosedLoopController pivotController = motor.getClosedLoopController();

    private static AcquisitionPivot instance = null;

    private AcquisitionPivot() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.apply(baseConfig).inverted(true).idleMode(IdleMode.kBrake).closedLoop
                .outputRange(-AcquisitionPivotConstants.POWER, AcquisitionPivotConstants.POWER)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(AcquisitionPivotConstants.P, AcquisitionPivotConstants.I,
                        AcquisitionPivotConstants.D)
                .allowedClosedLoopError(AcquisitionPivotConstants.ALLOWED_ERROR_DEGREES,
                        ClosedLoopSlot.kSlot0);
        pivotConfig.absoluteEncoder.positionConversionFactor(AcquisitionPivotConstants.ENCODER_CONVERSION_FACTOR)
                .inverted(true);

        motor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Gets the instance of the pivot. Instantiates the pivot if null.
     */
    public static AcquisitionPivot getInstance() {
        if (instance == null) {
            instance = new AcquisitionPivot();
        }
        return instance;
    }

    /**
     * Sets the pivot motor position.
     *
     * @param setpoint The setpoint for the pivot motor to go to.
     */
    public void setAngleSetpoint(PivotSetpoint setpoint) {
        setAngle(setpoint.getDegrees());
    }

    public void setAngle(double degrees) {
        pivotController.setSetpoint(
                MathUtil.clamp(degrees, AcquisitionPivotConstants.MIN_ANGLE, AcquisitionPivotConstants.MAX_ANGLE),
                ControlType.kPosition);
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivot() {
        motor.stopMotor();
    }

    /**
     * Gets if the pivot motor is at the setpoint.
     */
    public boolean isAtSetpoint() {
        return MathUtil.isNear(pivotController.getSetpoint(), getPosition(),
                AcquisitionPivotConstants.OPERATING_TOLERANCE);
    }

    /**
     * Gets the position of the pivot encoder.
     */
    public double getPosition() {
        return pivotEncoder.getPosition();
    }
}
