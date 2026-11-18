package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.NeoMotorConstants;

public class Feeder extends SubsystemBase {
    private static Feeder instance;
    private final SparkFlex motor = new SparkFlex(FeederConstants.CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private Feeder() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(FeederConstants.P, FeederConstants.I, FeederConstants.D).feedForward
                .kV(FeederConstants.V);
        config.closedLoop.maxMotion.maxAcceleration(FeederConstants.MAX_ACCELERATION);
        config.encoder.quadratureAverageDepth(5).quadratureMeasurementPeriod(10);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Feeder getInstance() {
        if (instance == null) {
            instance = new Feeder();
        }
        return instance;
    }

    /**
     * Runs the feeder motor forward at the RPM in
     * {@link Dashboard#MANUAL_FEEDER_RPM}.
     */
    public void start() {
        controller.setSetpoint(Dashboard.MANUAL_FEEDER_RPM.get(), ControlType.kVelocity);
    }

    /**
     * Runs the feeder motor in reverse at the RPM in
     * {@link FeederConstants#REVERSE_RPM}.
     */
    public void reverse() {
        controller.setSetpoint(FeederConstants.REVERSE_RPM, ControlType.kVelocity);
    }

    /**
     * Stops the feeder.
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Returns the current feeder speed.
     *
     * @return Current feeder speed in RPM.
     */
    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * Returns the target feeder speed.
     *
     * @return Target feeder speed in RPM.
     */
    public double getTargetRPM() {
        return controller.getSetpoint();
    }
}
