package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final SparkFlex firstShooter1 = new SparkFlex(ShooterConstants.FIRST_TOP_SHOOTER_CAN_ID,
            MotorType.kBrushless);
    private final SparkFlex firstShooter2 = new SparkFlex(ShooterConstants.FIRST_BOTTOM_SHOOTER_CAN_ID,
            MotorType.kBrushless);
    private final SparkFlex secondShooter1 = new SparkFlex(ShooterConstants.SECOND_TOP_SHOOTER_CAN_ID,
            MotorType.kBrushless);
    private final SparkFlex secondShooter2 = new SparkFlex(ShooterConstants.SECOND_BOTTOM_SHOOTER_CAN_ID,
            MotorType.kBrushless);

    private final SparkClosedLoopController firstController = firstShooter1.getClosedLoopController();
    private final SparkClosedLoopController secondController = secondShooter1.getClosedLoopController();
    private final RelativeEncoder firstEncoder = firstShooter1.getEncoder();
    private final RelativeEncoder secondEncoder = secondShooter1.getEncoder();

    private static Shooter instance;

    private Shooter() {
        SparkFlexConfig baseConfig = new SparkFlexConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
                .outputRange(NeoMotorConstants.MIN_POWER, NeoMotorConstants.MAX_POWER).feedForward
                .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);
        SparkFlexConfig firstConfig1 = new SparkFlexConfig();
        firstConfig1.apply(baseConfig);
        firstShooter1.configure(firstConfig1, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        SparkFlexConfig firstConfig2 = new SparkFlexConfig();
        firstConfig2.apply(baseConfig).follow(firstShooter1, true);
        firstShooter2.configure(firstConfig2, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        SparkFlexConfig secondConfig1 = new SparkFlexConfig();
        secondConfig1.apply(baseConfig);
        secondShooter1.configure(secondConfig1, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        SparkFlexConfig secondConfig2 = new SparkFlexConfig();
        secondConfig2.apply(baseConfig).follow(secondShooter1, true);
        secondShooter2.configure(secondConfig2, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public void runFirstShooter(double rpm) {
        firstController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void runSecondShooter(double rpm) {
        secondController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void stopFirstShooter() {
        firstShooter1.stopMotor();
    }

    public void stopSecondShooter() {
        secondShooter1.stopMotor();
    }

    public double getFirstShooterRPM() {
        return firstEncoder.getVelocity();
    }

    public double getSecondShooterRPM() {
        return secondEncoder.getVelocity();
    }
}
