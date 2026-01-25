package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final SparkMax firstTopShooter = new SparkMax(ShooterConstants.FIRST_TOP_SHOOTER_CAN_ID,
            MotorType.kBrushless);
    private final SparkMax firstBottomShooter = new SparkMax(ShooterConstants.FIRST_BOTTOM_SHOOTER_CAN_ID,
            MotorType.kBrushless);
    private final SparkMax secondTopShooter = new SparkMax(ShooterConstants.SECOND_TOP_SHOOTER_CAN_ID,
            MotorType.kBrushless);
    private final SparkMax secondBottomShooter = new SparkMax(ShooterConstants.SECOND_BOTTOM_SHOOTER_CAN_ID,
            MotorType.kBrushless);

    private static Shooter instance;

    private Shooter() {
        firstTopShooter.configure(SparkMaxConfig.Presets.REV_NEO, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        firstBottomShooter.configure(SparkMaxConfig.Presets.REV_NEO, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        secondTopShooter.configure(SparkMaxConfig.Presets.REV_NEO, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        secondBottomShooter.configure(SparkMaxConfig.Presets.REV_NEO, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public void runFirstShooter(double speed) {
        firstTopShooter.set(speed);
        firstBottomShooter.set(speed);
    }

    public void runSecondShooter(double speed) {
        secondTopShooter.set(speed);
        secondBottomShooter.set(speed);
    }

    public void stopFirstShooter() {
        firstTopShooter.stopMotor();
        firstBottomShooter.stopMotor();
    }

    public void stopSecondShooter() {
        secondTopShooter.stopMotor();
        secondBottomShooter.stopMotor();
    }
}
