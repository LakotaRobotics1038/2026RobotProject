package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.AcquisitionConstants.Setpoint;

public class Acquisition extends SubsystemBase {
    private final SparkMax pivot = new SparkMax(AcquisitionConstants.PIVOT_CAN_ID, MotorType.kBrushless);
    private final SparkMax intake = new SparkMax(AcquisitionConstants.INTAKE_CAN_ID, MotorType.kBrushless);

    private final SparkClosedLoopController pivotController = pivot.getClosedLoopController();
    private final SparkClosedLoopController intakeController = intake.getClosedLoopController();

    private Setpoint setpoint = Setpoint.RAISED;

    private static Acquisition instance = null;

    private Acquisition() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.apply(baseConfig).closedLoop
                .pid(AcquisitionConstants.PIVOT_P, AcquisitionConstants.PIVOT_I,
                        AcquisitionConstants.PIVOT_D).feedForward
                .sva(AcquisitionConstants.PIVOT_S, AcquisitionConstants.PIVOT_V, AcquisitionConstants.PIVOT_A);
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.apply(baseConfig).closedLoop
                .pid(AcquisitionConstants.INTAKE_P, AcquisitionConstants.INTAKE_I,
                        AcquisitionConstants.INTAKE_D).feedForward
                .sva(AcquisitionConstants.INTAKE_S, AcquisitionConstants.INTAKE_V, AcquisitionConstants.INTAKE_A);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Acquisition getInstance() {
        if (instance == null) {
            instance = new Acquisition();
        }
        return instance;
    }

    @Override
    public void periodic() {
        if (!intakeRequirements()) {
            stopIntake();
        }
    }

    public void raise() {
        pivotController.setSetpoint(AcquisitionConstants.RAISED_DEGREES, ControlType.kPosition);
        setpoint = Setpoint.RAISED;
    }

    public void lower() {
        pivotController.setSetpoint(AcquisitionConstants.LOWERED_DEGREES, ControlType.kPosition);
        setpoint = Setpoint.LOWERED;
    }

    public void acquire() {
        if (intakeRequirements()) {
            intakeController.setSetpoint(AcquisitionConstants.INTAKE_ACQUIRE_RPM, ControlType.kVelocity);
        }
    }

    public void dispose() {
        if (intakeRequirements()) {
            intakeController.setSetpoint(AcquisitionConstants.INTAKE_DISPOSE_RPM, ControlType.kVelocity);
        }
    }

    public void stopIntake() {
        intake.stopMotor();
    }

    public boolean atSetpoint() {
        return pivotController.isAtSetpoint();
    }

    public Setpoint getSetpoint() {
        return setpoint;
    }

    private boolean intakeRequirements() {
        return getSetpoint() == Setpoint.LOWERED && atSetpoint();
    }
}
