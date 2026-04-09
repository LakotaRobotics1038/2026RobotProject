package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.AcquisitionConstants;

public class Acquisition extends SubsystemBase {
    private static Acquisition instance;
    private final SparkFlex leftMotor = new SparkFlex(AcquisitionConstants.LEFT_MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkFlex rightMotor = new SparkFlex(AcquisitionConstants.RIGHT_MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();

    private Acquisition() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(SparkFlexConfig.IdleMode.kCoast).closedLoop.pid(AcquisitionConstants.P, AcquisitionConstants.I,
                AcquisitionConstants.D).feedForward
                .sva(AcquisitionConstants.S, AcquisitionConstants.V, AcquisitionConstants.A);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.apply(config).follow(leftMotor, true);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        leftMotor.stopMotor();
    }

    public double getRPM() {
        return leftMotor.getEncoder().getVelocity();
    }
}
