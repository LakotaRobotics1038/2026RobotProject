package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.AcquisitionConstants;

public class Acquisition extends SubsystemBase {

    private SparkFlex motor = new SparkFlex(AcquisitionConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private SparkClosedLoopController controller = motor.getClosedLoopController();
    private static Acquisition instance;

    private Acquisition() {
        SparkFlexConfig baseConfig = new SparkFlexConfig();
        baseConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(AcquisitionConstants.P, AcquisitionConstants.I,
                        AcquisitionConstants.D);
        motor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Acquisition getInstance() {
        if (instance == null) {
            instance = new Acquisition();
        }
        return instance;
    }

    /**
     * Runs the acquisition motor forward at the speed in
     * {@link AcquisitionConstants.ACQUIRE_SPEED}.
     */
    public void intake() {
        controller.setSetpoint(AcquisitionConstants.ACQUIRE_SPEED, ControlType.kDutyCycle);
    }

    /**
     * Runs the acquisition motor in reverse at the speed in
     * {@link AcquisitionConstants.DISPOSE_SPEED}.
     */
    public void dispose() {
        controller.setSetpoint(AcquisitionConstants.DISPOSE_SPEED, ControlType.kDutyCycle);
    }

    /**
     * Stops the acquisition.
     */
    public void stop() {
        motor.stopMotor();
    }

}
