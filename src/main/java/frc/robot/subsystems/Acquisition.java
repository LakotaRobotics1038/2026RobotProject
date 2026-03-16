package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;
import frc.robot.constants.NeoMotorConstants;

public class Acquisition extends SubsystemBase {
    private final SparkMax pivotMotor = new SparkMax(AcquisitionConstants.PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(AcquisitionConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

    private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
    private final SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

    private static Acquisition instance = null;

    private Acquisition() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.apply(baseConfig).inverted(true).idleMode(IdleMode.kBrake).closedLoop
                .outputRange(-AcquisitionConstants.PIVOT_POWER, AcquisitionConstants.PIVOT_POWER)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(AcquisitionConstants.PIVOT_P, AcquisitionConstants.PIVOT_I,
                        AcquisitionConstants.PIVOT_D)
                .allowedClosedLoopError(AcquisitionConstants.PIVOT_ALLOWED_ERROR_DEGREES,
                        ClosedLoopSlot.kSlot0);
        pivotConfig.absoluteEncoder.positionConversionFactor(AcquisitionConstants.PIVOT_ENCODER_CONVERSION_FACTOR)
                .inverted(true);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.apply(baseConfig).idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Gets the instance of Acquisition. Instantiates Acquisition if null.
     */
    public static Acquisition getInstance() {
        if (instance == null) {
            instance = new Acquisition();
        }
        return instance;
    }

    /**
     * Sets the pivot motor position.
     *
     * @param setpoint The setpoint for the pivot motor to go to.
     */
    public void setPivot(AcquisitionSetpoint setpoint) {
        setPivotDegrees(setpoint.getDegrees());
    }

    public void setPivotDegrees(double degrees) {
        pivotController.setSetpoint(degrees, ControlType.kPosition);
    }

    /**
     * Sets the Acquisition's intake power to
     * {@link AcquisitionConstants#INTAKE_ACQUIRE_DUTY_CYCLE}.
     */
    public void acquire() {
        intakeController.setSetpoint(AcquisitionConstants.INTAKE_ACQUIRE_DUTY_CYCLE, ControlType.kDutyCycle);
    }

    /**
     * Sets the Acquisition's disposal power to
     * {@link AcquisitionConstants#INTAKE_DISPOSE_DUTY_CYCLE}.
     */
    public void dispose() {
        intakeController.setSetpoint(AcquisitionConstants.INTAKE_DISPOSE_DUTY_CYCLE, ControlType.kDutyCycle);
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    /**
     * Gets if the pivot motor is at the setpoint.
     */
    public boolean pivotAtSetpoint() {
        return pivotController.isAtSetpoint();
    }

    public double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    /**
     * Gets the position of the pivot encoder.
     */
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }
}
