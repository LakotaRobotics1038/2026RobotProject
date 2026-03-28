package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ExtensionConstants;

public class HopperExtension extends SubsystemBase {
    private static HopperExtension instance;
    private SparkMax motor = new SparkMax(ExtensionConstants.MOTOR_CAN_ID,
            SparkFlex.MotorType.kBrushless);
    private SparkClosedLoopController controller = motor.getClosedLoopController();

    private HopperExtension() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).limitSwitch
                .forwardLimitSwitchType(Type.kNormallyOpen)
                .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
                .reverseLimitSwitchType(Type.kNormallyOpen)
                .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static HopperExtension getInstance() {
        if (instance == null) {
            instance = new HopperExtension();
        }
        return instance;
    }

    public void forward() {
        controller.setSetpoint(ExtensionConstants.FORWARD_POWER, ControlType.kDutyCycle);
    }

    public void backward() {
        controller.setSetpoint(ExtensionConstants.BACKWARD_POWER, ControlType.kDutyCycle);
    }

    public void stop() {
        controller.setSetpoint(0, ControlType.kDutyCycle);
    }

    public boolean getForwardLimitSwitchPressed() {
        return motor.getForwardLimitSwitch().isPressed();
    }

    public boolean getReverseLimitSwitchPressed() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }
}