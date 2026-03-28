package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterHoodConstants;

public class ShooterHood extends SubsystemBase {
    private static ShooterHood instance;

    private final SparkMax leftMotor = new SparkMax(ShooterHoodConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ShooterHoodConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();
    private final AbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();

    private ShooterHood() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.idleMode(SparkMaxConfig.IdleMode.kBrake).closedLoop.pid(ShooterHoodConstants.P,
                ShooterHoodConstants.I, ShooterHoodConstants.D).feedForward
                .sva(ShooterHoodConstants.S, ShooterHoodConstants.V, ShooterHoodConstants.A);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.apply(baseConfig).inverted(true).absoluteEncoder
                .positionConversionFactor(ShooterHoodConstants.HOOD_ENCODER_CONVERSION_FACTOR);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.apply(baseConfig).follow(leftMotor, true);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static ShooterHood getInstance() {
        if (instance == null) {
            instance = new ShooterHood();
        }
        return instance;
    }

    public void setAngle(double angle) {
        controller.setSetpoint(angle, ControlType.kPosition);
    }

    public double getAngle() {
        return encoder.getPosition();
    }
}
