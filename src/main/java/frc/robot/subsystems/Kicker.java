package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.NeoMotorConstants;

public class Kicker extends SubsystemBase {
    private final SparkFlex motor = new SparkFlex(KickerConstants.KICKER_CAN_ID, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private static Kicker instance;

    private Kicker() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).inverted(true)
                .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(KickerConstants.KICKER_P, KickerConstants.KICKER_I, KickerConstants.KICKER_D).feedForward
                .kV(KickerConstants.KICKER_V);
        config.encoder.quadratureAverageDepth(5).quadratureMeasurementPeriod(10);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Kicker getInstance() {
        if (instance == null) {
            instance = new Kicker();
        }
        return instance;
    }

    public void start() {
        controller.setSetpoint(Dashboard.MANUAL_KICKER_RPM.get(), ControlType.kVelocity);
    }

    public void reverse() {
        controller.setSetpoint(KickerConstants.KICKER_REVERSE_RPM, ControlType.kVelocity);
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
