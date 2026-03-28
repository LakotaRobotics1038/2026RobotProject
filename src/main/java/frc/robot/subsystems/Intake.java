package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private final SparkFlex leftMotor = new SparkFlex(IntakeConstants.LEFT_MOTOR_CAN_ID,
            SparkFlex.MotorType.kBrushless);
    private final SparkFlex rightMotor = new SparkFlex(IntakeConstants.RIGHT_MOTOR_CAN_ID,
            SparkFlex.MotorType.kBrushless);
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();

    private Intake() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(SparkFlexConfig.IdleMode.kCoast).closedLoop.pid(IntakeConstants.P, IntakeConstants.I,
                IntakeConstants.D).feedForward.sva(IntakeConstants.S, IntakeConstants.V, IntakeConstants.A);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.apply(config).follow(leftMotor, true);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void intake() {
        controller.setSetpoint(IntakeConstants.INTAKE_RPM, ControlType.kVelocity);
    }

    public void dispose() {
        controller.setSetpoint(IntakeConstants.DISPOSE_RPM, ControlType.kVelocity);
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    public double getRPM() {
        return leftMotor.getEncoder().getVelocity();
    }
}
