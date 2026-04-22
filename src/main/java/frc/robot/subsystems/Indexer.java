package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.NeoMotorConstants;

public class Indexer extends SubsystemBase {
    private static Indexer instance;

    private final SparkFlex motor = new SparkFlex(IndexerConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private Indexer() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    public void intake() {
        controller.setSetpoint(IndexerConstants.FORWARD_POWER, ControlType.kDutyCycle);
    }

    public void dispose() {
        controller.setSetpoint(IndexerConstants.BACKWARD_POWER, ControlType.kDutyCycle);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }
}
