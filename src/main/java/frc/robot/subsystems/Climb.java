package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final SparkMax climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController climbController = climbMotor.getClosedLoopController();
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
        climbController.setSetpoint(ClimbConstants.MAX_CLIMB, ControlType.kPosition);
    }

    public void climbDown() {
        climbController.setSetpoint(ClimbConstants.MIN_CLIMB, ControlType.kPosition);
    }

    public void stopClimb() {
        climbMotor.stopMotor();
    }
}