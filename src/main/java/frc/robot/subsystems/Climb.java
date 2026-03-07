package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ClimbConstants.ClimbSetpoint;
import frc.robot.constants.NeoMotorConstants;

public class Climb extends SubsystemBase {

    private final SparkMax motor = new SparkMax(ClimbConstants.MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private static Climb instance;

    private Climb() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT).closedLoop
                .pid(ClimbConstants.P, ClimbConstants.I, ClimbConstants.D);
        config.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition);
        config.encoder.positionConversionFactor(ClimbConstants.CLIMB_POSITION_CONVERSION_FACTOR);
        motor.configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    public void setSetpoint(ClimbSetpoint setpoint) {
        controller.setSetpoint(setpoint.getSetpoint(),  ControlType.kPosition);
    }

    public boolean isAtSetpoint() {
        return controller.isAtSetpoint();
    }

    public double getSetpoint() {
        return controller.getSetpoint();
    }

    public void stopClimb() {
        motor.stopMotor();
    }
}