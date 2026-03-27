package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    private static Indexer instance;

    private final SparkFlex motor = new SparkFlex(IndexerConstants.MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);

    private Indexer() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    public void start() {
        motor.set(IndexerConstants.FORWARD_POWER);
    }

    public void backward() {
        motor.set(IndexerConstants.BACKWARD_POWER);
    }

    public void stop() {
        motor.stopMotor();
    }
}
