package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionConstants;

public class Acquisition extends SubsystemBase {
    private static Acquisition instance;
    private final SparkFlex motor = new SparkFlex(AcquisitionConstants.MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private Acquisition() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(SparkFlexConfig.IdleMode.kCoast).closedLoop.pid(AcquisitionConstants.P, AcquisitionConstants.I,
                AcquisitionConstants.D).feedForward
                .sva(AcquisitionConstants.S, AcquisitionConstants.V, AcquisitionConstants.A);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Acquisition getInstance() {
        if (instance == null) {
            instance = new Acquisition();
        }
        return instance;
    }

    public void intake() {
        controller.setSetpoint(AcquisitionConstants.INTAKE_RPM, ControlType.kVelocity);
    }

    public void dispose() {
        controller.setSetpoint(AcquisitionConstants.DISPOSE_RPM, ControlType.kVelocity);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }
}
