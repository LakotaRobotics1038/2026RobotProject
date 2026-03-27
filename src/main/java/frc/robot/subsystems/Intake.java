package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private final SparkFlex motor = new SparkFlex(IntakeConstants.MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);

    private Intake() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(SparkFlexConfig.IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void forward() {
        motor.set(IntakeConstants.FORWARD_POWER);
    }

    public void backward() {
        motor.set(IntakeConstants.BACKWARD_POWER);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }
}
