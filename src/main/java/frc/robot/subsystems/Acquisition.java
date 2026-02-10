package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.NeoMotorConstants;

public class Acquisition {
    private final SparkMax pivot = new SparkMax(AcquisitionConstants.PIVOT_CAN_ID, MotorType.kBrushless);
    private final SparkMax intake = new SparkMax(0, MotorType.kBrushless);

    private final SparkClosedLoopController pivotController = pivot.getClosedLoopController();
    private final SparkClosedLoopController intakeController = intake.getClosedLoopController();

    private final AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();
    private final RelativeEncoder intakeEncoder = intake.getEncoder();

    private static Acquisition instance = null;

    private Acquisition() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT).closedLoop
                .outputRange(NeoMotorConstants.MIN_POWER, NeoMotorConstants.MAX_POWER);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.apply(baseConfig);
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
}
