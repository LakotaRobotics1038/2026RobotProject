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
    private final SparkFlex motor = new SparkFlex(FeederConstants.FEEDER_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private Feeder() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(FeederConstants.FEEDER_P, FeederConstants.FEEDER_I, FeederConstants.FEEDER_D).feedForward
                .kV(FeederConstants.FEEDER_V);
        config.encoder.quadratureAverageDepth(5).quadratureMeasurementPeriod(10);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Feeder getInstance() {
        if (instance == null) {
            instance = new Feeder();
        }
        return instance;
    }

    public void start() {
        controller.setSetpoint(Dashboard.MANUAL_FEEDER_RPM.get(), ControlType.kVelocity);
    }

    public void reverse() {
        controller.setSetpoint(FeederConstants.FEEDER_REVERSE_RPM, ControlType.kVelocity);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }

    public double getTargetRPM() {
        return controller.getSetpoint();
    }
}
