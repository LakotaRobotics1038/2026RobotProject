package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AcquisitionPivotConstants;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;
import frc.robot.constants.NeoMotorConstants;

public class AcquisitionPivot extends SubsystemBase {
    private static AcquisitionPivot instance;

    private final SparkMax motor = new SparkMax(AcquisitionPivotConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private final AbsoluteEncoder pivotEncoder = motor.getAbsoluteEncoder();
    private final PIDController pidController = new PIDController(AcquisitionPivotConstants.P,
            AcquisitionPivotConstants.I,
            AcquisitionPivotConstants.D);
    private boolean enabled = false;

    private AcquisitionPivot() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.smartCurrentLimit(NeoMotorConstants.MAX_NEO_CURRENT);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.apply(baseConfig).inverted(true).idleMode(IdleMode.kBrake);
        pivotConfig.absoluteEncoder.positionConversionFactor(AcquisitionPivotConstants.ENCODER_CONVERSION_FACTOR)
                .inverted(true);

        motor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController.setTolerance(AcquisitionPivotConstants.ALLOWED_ERROR_DEGREES);
    }

    /**
     * Gets the instance of the pivot. Instantiates the pivot if null.
     */
    public static AcquisitionPivot getInstance() {
        if (instance == null) {
            instance = new AcquisitionPivot();
        }
        return instance;
    }

    /**
     * Sets the pivot motor position.
     *
     * @param setpoint The setpoint for the pivot motor to go to.
     */
    public void setAngleSetpoint(PivotSetpoint setpoint) {
        setAngle(setpoint.getDegrees());
    }

    public void setAngle(double degrees) {
        setEnabled(true);
        pidController.setSetpoint(
                MathUtil.clamp(degrees, AcquisitionPivotConstants.MIN_ANGLE, AcquisitionPivotConstants.MAX_ANGLE));
    }

    @Override
    public void periodic() {
        if (enabled) {
            double output = MathUtil.clamp(pidController.calculate(getPosition()),
                    -AcquisitionPivotConstants.POWER, AcquisitionPivotConstants.POWER);
            motor.set(output);
        } else {
            stopPivot();
        }
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivot() {
        motor.stopMotor();
    }

    /**
     * Gets if the pivot motor is at the setpoint.
     */
    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }

    /**
     * Gets the position of the pivot encoder.
     */
    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
