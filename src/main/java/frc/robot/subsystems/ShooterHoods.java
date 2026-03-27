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
import frc.robot.constants.ShooterHoodsConstants;

public class ShooterHoods extends SubsystemBase {
    private static ShooterHoods instance;

    private final SparkMax leftMotor = new SparkMax(ShooterHoodsConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ShooterHoodsConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();
    private final AbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();

    private ShooterHoods() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.idleMode(SparkMaxConfig.IdleMode.kBrake).closedLoop.pid(ShooterHoodsConstants.P,
                ShooterHoodsConstants.I, ShooterHoodsConstants.D).feedForward
                .sva(ShooterHoodsConstants.S, ShooterHoodsConstants.V, ShooterHoodsConstants.A);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.apply(baseConfig).inverted(true).absoluteEncoder
                .positionConversionFactor(ShooterHoodsConstants.HOOD_ENCODER_CONVERSION_FACTOR);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.apply(baseConfig).follow(leftMotor, true);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static ShooterHoods getInstance() {
        if (instance == null) {
            instance = new ShooterHoods();
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
