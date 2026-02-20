package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.NeoMotorConstants;

public class Kicker extends SubsystemBase {
    private final SparkMax motor = new SparkMax(KickerConstants.CAN_ID, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();
    private static Kicker instance = null;

    private Kicker() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT).closedLoop
                .pid(KickerConstants.P, KickerConstants.I, KickerConstants.D).feedForward
                .sva(KickerConstants.S, KickerConstants.V, KickerConstants.A);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Kicker getInstance() {
        if (instance == null) {
            instance = new Kicker();
        }
        return instance;
    }

    public void start(double rpm) {
        controller.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public double getTargetRPM() {
        return controller.getSetpoint();
    }

    public boolean isAtTargetRPM() {
        return controller.isAtSetpoint();
    }
}
