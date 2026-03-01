package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

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
                .pid(AcquisitionConstants.PIVOT_P, AcquisitionConstants.PIVOT_I,
                        AcquisitionConstants.PIVOT_D)
                .allowedClosedLoopError(AcquisitionConstants.PIVOT_ALLOWED_ERROR_DEGREES,
                        ClosedLoopSlot.kSlot0).feedForward
                .sva(AcquisitionConstants.PIVOT_S, AcquisitionConstants.PIVOT_V, AcquisitionConstants.PIVOT_A);
        pivotConfig.absoluteEncoder.positionConversionFactor(AcquisitionConstants.PIVOT_ENCODER_CONVERSION_FACTOR);

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
     * Sets the Acquisition's pivot position to {@link AcquisitionSetpoint#RAISED}.
     */
    public void raise() {
        setpoint = AcquisitionSetpoint.RAISED;
        pivotController.setSetpoint(setpoint.getDegrees(), ControlType.kPosition);
    }

    /**
     * Sets the Acquisition's pivot position to {@link AcquisitionSetpoint#LOWERED}.
     */
    public void lower() {
        setpoint = AcquisitionSetpoint.LOWERED;
        pivotController.setSetpoint(setpoint.getDegrees(), ControlType.kPosition);
    }

    /**
     * Sets the Acquisition's intake RPM to {@link AcquisitionSetpoint#INTAKE_ACQUIRE_RPM}.
     */
    public void acquire() {
        if (readyToIntake()) {
            intakeController.setSetpoint(AcquisitionConstants.INTAKE_ACQUIRE_RPM, ControlType.kVelocity);
        } else {
            stopIntake();
        }
    }

    /**
     * Sets the Acquisition's intake RPM to {@link AcquisitionSetpoint#INTAKE_DISPOSE_RPM}.
     */
    public void dispose() {
        if (readyToIntake()) {
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
    public boolean readyToIntake() {
        return setpoint == AcquisitionSetpoint.LOWERED && atSetpoint();
    }
}
