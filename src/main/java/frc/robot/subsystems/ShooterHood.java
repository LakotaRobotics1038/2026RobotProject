package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterHoodConstants;

public class ShooterHood extends SubsystemBase {
    private static ShooterHood instance;

    private final SparkMax leftMotor = new SparkMax(ShooterHoodConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ShooterHoodConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();
    private final AbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();

    private ShooterHood() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_550_CURRENT)
                .idleMode(SparkMaxConfig.IdleMode.kBrake).closedLoop.pid(ShooterHoodConstants.P,
                        ShooterHoodConstants.I, ShooterHoodConstants.D);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.apply(baseConfig).inverted(true).absoluteEncoder.inverted(true)
                .positionConversionFactor(ShooterHoodConstants.HOOD_ENCODER_CONVERSION_FACTOR);
        leftMotorConfig.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(0,
                ShooterHoodConstants.HOOD_ENCODER_CONVERSION_FACTOR).feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .allowedClosedLoopError(ShooterHoodConstants.ANGLE_TOLERANCE, ClosedLoopSlot.kSlot0);

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
        controller.setSetpoint(
                ShooterHoodConstants.SHOOTER_FULL_RETRACTION_ANGLE
                        - MathUtil.clamp(angle, ShooterHoodConstants.SHOOTER_NO_RETRACTION_ANGLE,
                                ShooterHoodConstants.SHOOTER_FULL_RETRACTION_ANGLE),
                ControlType.kPosition);
    }

    public double getAngle() {
        return encoder.getPosition() + ShooterHoodConstants.SHOOTER_NO_RETRACTION_ANGLE;
    }
}
