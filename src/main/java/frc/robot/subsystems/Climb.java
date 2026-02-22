package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final SparkMax climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_CAN_ID, MotorType.kBrushless);
    private final AbsoluteEncoder climbEncoder = climbMotor.getAbsoluteEncoder();
    private static Climb instance;
    private double position;

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
        while (position < ClimbConstants.MAX) {
            climbMotor.set(ClimbConstants.CLIMB_MOTOR_SPEED_UP);
        }
    }

    public void climbDown() {
        while (position > 0) {
            climbMotor.set(-(ClimbConstants.CLIMB_MOTOR_SPEED_DOWN));
        }
    }

    public void stopClimb() {
        climbMotor.stopMotor();
    }
}