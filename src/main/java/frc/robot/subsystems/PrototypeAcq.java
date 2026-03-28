package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.PrototypeAcqConstants;

public class PrototypeAcq {

    private SparkFlex motor = new SparkFlex(PrototypeAcqConstants.ACQ_MOTOR_CAN_ID, MotorType.kBrushless);
    private SparkClosedLoopController controller = motor.getClosedLoopController();
    private RelativeEncoder encoder = motor.getEncoder();
    private static PrototypeAcq instance;

    private PrototypeAcq() {
        SparkFlexConfig baseConfig = new SparkFlexConfig();
        baseConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(PrototypeAcqConstants.ACQUISITION_P, PrototypeAcqConstants.ACQUISITION_I,
                        PrototypeAcqConstants.ACQUISITION_D);
        motor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static PrototypeAcq getInstance() {
        if (instance == null) {
            instance = new PrototypeAcq();
        }
        return instance;
    }

    public void start() {
        motor.set(PrototypeAcqConstants.ACQUISITION_ACQUIRE_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }

}
