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
        config.smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).idleMode(IdleMode.kCoast)
                .inverted(true).closedLoop
                .pid(IndexerConstants.P, IndexerConstants.I, IndexerConstants.D).feedForward
                .kV(IndexerConstants.V);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    /**
     * Runs the indexer motor forward at the RPM in
     * {@link Dashboard#MANUAL_INDEXER_RPM}.
     */
    public void intake() {
        controller.setSetpoint(Dashboard.MANUAL_INDEXER_RPM.get(), ControlType.kVelocity);
    }

    /**
     * Runs the indexer motor in reverse at the RPM in
     * {@link IndexerConstants#BACKWARD_RPM}.
     */
    public void dispose() {
        controller.setSetpoint(IndexerConstants.BACKWARD_RPM, ControlType.kVelocity);
    }

    /**
     * Stops the indexer.
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Returns the current indexer speed.
     *
     * @return Current indexer speed in RPM.
     */
    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * Returns the target indexer speed.
     *
     * @return Target indexer speed in RPM.
     */
    public double getTargetRPM() {
        return controller.getSetpoint();
    }
}
