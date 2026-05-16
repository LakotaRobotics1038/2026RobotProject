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

public class HopperExtension extends SubsystemBase {
    private static HopperExtension instance;
    private final SparkMax motor = new SparkMax(HopperExtensionConstants.MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private HopperExtension() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(HopperExtensionConstants.CURRENT_LIMIT).idleMode(IdleMode.kCoast).limitSwitch
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

    public void setSpeed(double dutyCycle) {
        controller.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void out() {
        setSpeed(HopperExtensionConstants.OUT_DUTY_CYCLE);
    }

    public void in() {
        setSpeed(HopperExtensionConstants.IN_DUTY_CYCLE);
    }

    public void inSlow() {
        setSpeed(Dashboard.HOPPER_SLOW_SHOOT_DUTY_CYCLE.get());
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