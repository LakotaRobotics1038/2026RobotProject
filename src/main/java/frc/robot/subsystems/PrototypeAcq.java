package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.PrototypeAcqConstants;

public class PrototypeAcq extends SubsystemBase {

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

    public void intake() {
        controller.setSetpoint(PrototypeAcqConstants.ACQUISITION_ACQUIRE_SPEED, ControlType.kDutyCycle);
    }

    public void dispose() {
        controller.setSetpoint(PrototypeAcqConstants.ACQUISITION_DISPOSE_SPEED, ControlType.kDutyCycle);
    }

    public void stop() {
        motor.stopMotor();
    }

}
