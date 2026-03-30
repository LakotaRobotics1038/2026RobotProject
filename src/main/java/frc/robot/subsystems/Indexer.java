package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.NeoMotorConstants;

public class Indexer extends SubsystemBase {
    private static Indexer instance;

    private final SparkMax motor = new SparkMax(IndexerConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    private Indexer() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the indexer motor forward at the speed in
     * {@link IndexerConstants#ACQUIRE_DUTY_CYCLE}.
     */
    public void start() {
        controller.setSetpoint(IndexerConstants.ACQUIRE_DUTY_CYCLE, ControlType.kDutyCycle);
    }

    /**
     * Runs the indexer motor in reverse at the speed in
     * {@link IndexerConstants#DISPOSE_DUTY_CYCLE}.
     */
    public void reverse() {
        controller.setSetpoint(IndexerConstants.DISPOSE_DUTY_CYCLE, ControlType.kDutyCycle);
    }

    /**
     * Stops the indexer.
     */
    public void stop() {
        motor.stopMotor();
    }
}
