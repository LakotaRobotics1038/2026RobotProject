package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final SparkMax motor = new SparkMax(ClimbConstants.CLIMB_MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private static Climb instance;

    private Climb() {

        motor.configure(SparkMaxConfig.Presets.REV_NEO, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    public void climbUp() {
        controller.setSetpoint(ClimbConstants.MAX_CLIMB, ControlType.kPosition);
    }

    public void climbDown() {
        controller.setSetpoint(ClimbConstants.MIN_CLIMB, ControlType.kPosition);
    }

    public boolean isAtSetpoint() {
        return controller.isAtSetpoint();
    }

    public void stopClimb() {
        motor.stopMotor();
    }
}