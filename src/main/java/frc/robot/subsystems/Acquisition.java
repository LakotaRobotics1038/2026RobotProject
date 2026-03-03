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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;
import frc.robot.constants.NeoMotorConstants;

public class Acquisition extends SubsystemBase {
    private final SparkMax pivotMotor = new SparkMax(AcquisitionConstants.PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(AcquisitionConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

    private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
    private final SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

    private AcquisitionSetpoint setpoint = AcquisitionSetpoint.RAISED;

    private static Acquisition instance = null;

    private Acquisition() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.apply(baseConfig).closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(AcquisitionConstants.PIVOT_P, AcquisitionConstants.PIVOT_I,
                        AcquisitionConstants.PIVOT_D)
                .allowedClosedLoopError(AcquisitionConstants.PIVOT_ALLOWED_ERROR_DEGREES,
                        ClosedLoopSlot.kSlot0);
        pivotConfig.absoluteEncoder.positionConversionFactor(AcquisitionConstants.PIVOT_ENCODER_DEGREES_CONVERSION_FACTOR * AcquisitionConstants.PIVOT_GEAR_RATIO);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.apply(baseConfig).closedLoop
                .pid(AcquisitionConstants.INTAKE_P, AcquisitionConstants.INTAKE_I,
                        AcquisitionConstants.INTAKE_D).feedForward
                .sva(AcquisitionConstants.INTAKE_S, AcquisitionConstants.INTAKE_V, AcquisitionConstants.INTAKE_A);
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
        this.setpoint = setpoint;
        pivotController.setSetpoint(setpoint.getDegrees(), ControlType.kPosition);
    }

    /**
     * Sets the Acquisition's intake RPM to
     * {@link AcquisitionConstants#INTAKE_ACQUIRE_RPM} if the pivot is in the
     * correct position, otherwise stops the intake.
     */
    public void acquire() {
        if (isIntakeAllowed()) {
            intakeController.setSetpoint(AcquisitionConstants.INTAKE_ACQUIRE_RPM, ControlType.kVelocity);
        } else {
            stopIntake();
        }
    }

    /**
     * Sets the Acquisition's intake RPM to
     * {@link AcquisitionConstants#INTAKE_DISPOSE_RPM} if the pivot is in the
     * correct position. Otherwise, stops the intake motor.
     */
    public void dispose() {
        if (isIntakeAllowed()) {
            intakeController.setSetpoint(AcquisitionConstants.INTAKE_DISPOSE_RPM, ControlType.kVelocity);
        } else {
            stopIntake();
        }
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    /**
     * Gets if the pivot motor is at the setpoint.
     */
    public boolean atSetpoint() {
        return pivotController.isAtSetpoint();
    }

    /**
     * Gets the setpoint of the pivot motor.
     */
    public AcquisitionSetpoint getSetpoint() {
        return setpoint;
    }

    /**
     * Gets the position of the pivot encoder.
     */
    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    /**
     * Gets if the requirements are met for the intake motor to run.
     */
    private boolean isIntakeAllowed() {
        return setpoint == AcquisitionSetpoint.LOWERED && atSetpoint();
    }
}
