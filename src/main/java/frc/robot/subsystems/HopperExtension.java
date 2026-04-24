package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperExtensionConstants;
import frc.robot.constants.NeoMotorConstants;

public class HopperExtension extends SubsystemBase {
    private static HopperExtension instance;
    private final SparkMax motor = new SparkMax(HopperExtensionConstants.MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private HopperExtension() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT).idleMode(IdleMode.kBrake).limitSwitch
                .reverseLimitSwitchType(Type.kNormallyClosed)
                .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
        config.closedLoop.pid(HopperExtensionConstants.P, HopperExtensionConstants.I,
                HopperExtensionConstants.D);
        config.encoder.positionConversionFactor(1 / (HopperExtensionConstants.EXTENSION_TO_MOTOR_RATIO
                * HopperExtensionConstants.EXTENSION_GEARBOX));
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static HopperExtension getInstance() {
        if (instance == null) {
            instance = new HopperExtension();
        }
        return instance;
    }

    public void out() {
        controller.setSetpoint(1, ControlType.kPosition);
    }

    public void in() {
        controller.setSetpoint(HopperExtensionConstants.IN_DUTY_CYCLE, ControlType.kDutyCycle);
        System.out.println("Hopper Extension In");
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean getReverseLimitSwitchPressed() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public boolean isAtSetpoint() {
        return controller.isAtSetpoint();
    }
}