package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final SparkMax climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_CAN_ID, MotorType.kBrushless);

    private static Climb instance;

    private Climb() {
        climbMotor.configure(SparkMaxConfig.Presets.REV_NEO, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    public void climbUp() {
        climbMotor.set(ClimbConstants.CLIMB_MOTOR_SPEED);
    }

    public void climbDown() {
        climbMotor.set(-(ClimbConstants.CLIMB_MOTOR_SPEED));
    }

    public void stopClimb() {
        climbMotor.stopMotor();
    }
}