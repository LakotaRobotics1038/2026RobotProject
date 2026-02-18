package frc.robot.utils;

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

public abstract class Shooter extends SubsystemBase {
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;

    protected Shooter(int leftMotorCanId, int rightMotorCanId) {
        SparkFlexConfig baseConfig = new SparkFlexConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_VORTEX_CURRENT).closedLoop
                .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
                .outputRange(NeoMotorConstants.MIN_POWER, NeoMotorConstants.MAX_POWER).feedForward
                .sva(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);

        SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
        leftMotorConfig.apply(baseConfig);
        leftMotor = new SparkFlex(leftMotorCanId, MotorType.kBrushless);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig rightMotorConfig = new SparkFlexConfig();
        rightMotorConfig.apply(baseConfig).follow(leftMotor, true);
        rightMotor = new SparkFlex(rightMotorCanId, MotorType.kBrushless);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = leftMotor.getClosedLoopController();
        encoder = leftMotor.getEncoder();
    }

    protected void start(double rpm) {
        controller.setSetpoint(rpm, ControlType.kVelocity);
    }

    protected void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    protected double getRPM() {
        return encoder.getVelocity();
    }
}
