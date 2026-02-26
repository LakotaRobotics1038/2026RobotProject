package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class Acquisition extends SubsystemBase {
    private final SparkMax pivotMotor = new SparkMax(AcquisitionConstants.PIVOT_CAN_ID, MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(AcquisitionConstants.INTAKE_CAN_ID, MotorType.kBrushless);

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
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.apply(baseConfig).closedLoop
                .pid(AcquisitionConstants.INTAKE_P, AcquisitionConstants.INTAKE_I,
                        AcquisitionConstants.INTAKE_D).feedForward
                .sva(AcquisitionConstants.INTAKE_S, AcquisitionConstants.INTAKE_V, AcquisitionConstants.INTAKE_A);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Acquisition getInstance() {
        if (instance == null) {
            instance = new Acquisition();
        }
        return instance;
    }

    @Override
    public void periodic() {
        if (!readyToIntake()) {
            stopIntake();
        }
    }

    /**
     * Sets the Acquisition's pivot position to AcquisitionSetpoint.RAISED
     */
    public void raise() {
        setpoint = AcquisitionSetpoint.RAISED;
        pivotController.setSetpoint(setpoint.getDegrees(), ControlType.kPosition);
    }

    /**
     * Sets the Acquisition's pivot position to AcquisitionSetpoint.LOWERED
     */
    public void lower() {
        setpoint = AcquisitionSetpoint.LOWERED;
        pivotController.setSetpoint(setpoint.getDegrees(), ControlType.kPosition);
    }

    /**
     * Sets the Acquisition's intake RPM to
     * AcquisitionConstants.INTAKE_ACQUIRE_RPM
     */
    public void acquire() {
        if (readyToIntake()) {
            intakeController.setSetpoint(AcquisitionConstants.INTAKE_ACQUIRE_RPM, ControlType.kVelocity);
        }
    }

    /**
     * Sets the Acquisition's intake RPM to
     * AcquisitionConstants.INTAKE_DISPOSE_RPM
     */
    public void dispose() {
        if (readyToIntake()) {
            intakeController.setSetpoint(AcquisitionConstants.INTAKE_DISPOSE_RPM, ControlType.kVelocity);
        }
    }

    /**
     * Stops the intakeMotor
     */
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    /**
     * Gets the if the Acquisition's pivotMotor is at setpoint
     */
    public boolean atSetpoint() {
        return pivotController.isAtSetpoint();
    }

    public AcquisitionSetpoint getSetpoint() {
        return setpoint;
    }

    private boolean readyToIntake() {
        return getSetpoint() == AcquisitionSetpoint.LOWERED && atSetpoint();
    }
}
