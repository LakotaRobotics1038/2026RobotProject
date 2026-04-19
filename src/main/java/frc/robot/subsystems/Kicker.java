package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.NeoMotorConstants;

public class Kicker extends SubsystemBase {
    private final SparkFlex kickerMotor = new SparkFlex(KickerConstants.KICKER_CAN_ID, MotorType.kBrushless);
    private final SparkFlex feederMotor = new SparkFlex(KickerConstants.FEEDER_CAN_ID, MotorType.kBrushless);
    private final SparkClosedLoopController kickerController = kickerMotor.getClosedLoopController();
    private final SparkClosedLoopController feederController = feederMotor.getClosedLoopController();
    private static Kicker instance;

    private Kicker() {
        SparkFlexConfig kickerConfig = new SparkFlexConfig();
        kickerConfig.idleMode(IdleMode.kCoast).inverted(true)
                .smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT).closedLoop
                .pid(KickerConstants.KICKER_P, KickerConstants.KICKER_I, KickerConstants.KICKER_D);
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig feederConfig = new SparkFlexConfig();
        feederConfig.idleMode(IdleMode.kCoast).inverted(true)
                .smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT).closedLoop
                .pid(KickerConstants.FEEDER_P, KickerConstants.FEEDER_I, KickerConstants.FEEDER_D);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Kicker getInstance() {
        if (instance == null) {
            instance = new Kicker();
        }
        return instance;
    }

    public void start() {
        kickerController.setSetpoint(KickerConstants.KICKER_SHOOT_RPM, ControlType.kVelocity);
        feederController.setSetpoint(KickerConstants.FEEDER_SHOOT_RPM, ControlType.kVelocity);
    }

    public void reverse() {
        kickerController.setSetpoint(KickerConstants.KICKER_REVERSE_RPM, ControlType.kVelocity);
        feederController.setSetpoint(KickerConstants.FEEDER_REVERSE_RPM, ControlType.kVelocity);
    }

    public void stop() {
        kickerMotor.stopMotor();
        feederMotor.stopMotor();
    }
}
