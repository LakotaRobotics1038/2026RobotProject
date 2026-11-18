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

    /**
     * Runs the extension motor at a duty cycle. Positive values extend the
     * hopper and negative values retract it.
     *
     * @param dutyCycle Duty cycle from -1 to 1.
     */
    public void setSpeed(double dutyCycle) {
        controller.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    /**
     * Extends the hopper at {@link HopperExtensionConstants#OUT_DUTY_CYCLE}.
     */
    public void out() {
        setSpeed(HopperExtensionConstants.OUT_DUTY_CYCLE);
    }

    /**
     * Retracts the hopper at {@link HopperExtensionConstants#IN_DUTY_CYCLE}.
     */
    public void in() {
        setSpeed(HopperExtensionConstants.IN_DUTY_CYCLE);
    }

    /**
     * Retracts the hopper at the duty cycle in
     * {@link Dashboard#HOPPER_SLOW_SHOOT_DUTY_CYCLE}.
     */
    public void inSlow() {
        setSpeed(Dashboard.HOPPER_SLOW_SHOOT_DUTY_CYCLE.get());
    }

    /**
     * Stops the extension motor.
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Indicates whether the reverse limit switch is pressed, meaning the
     * hopper is fully retracted.
     *
     * @return Whether the reverse limit switch is pressed.
     */
    public boolean getReverseLimitSwitchPressed() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    /**
     * Returns the extension motor's encoder position.
     *
     * @return Encoder position in motor rotations.
     */
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    /**
     * Indicates whether the closed loop controller is at its setpoint.
     *
     * @return Whether the extension motor is at its setpoint.
     */
    public boolean isAtSetpoint() {
        return controller.isAtSetpoint();
    }
}