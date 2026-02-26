package frc.robot.constants;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
    public static record ShooterModuleOptions(
            int leftMotorCanId,
            int rightMotorCanId,
            Translation2d translation,
            ServoChannel.ChannelId servoChannelID,
            ServoChannelConfig.PulseRange servoPulseRange) {
    }

    public static final ShooterModuleOptions NEAR_SHOOTER_MODULE_OPTIONS = new ShooterModuleOptions(
            0,
            0,
            new Translation2d(),
            ServoChannel.ChannelId.kChannelId0,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final ShooterModuleOptions FAR_SHOOTER_MODULE_OPTIONS = new ShooterModuleOptions(
            0,
            0,
            new Translation2d(),
            ServoChannel.ChannelId.kChannelId0,
            new ServoChannelConfig.PulseRange(1000, 1500, 2000));

    public static final int SERVO_HUB_CAN_ID = 0;

    public static final double RPM_TOLERANCE = 25;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;

    public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2);
    public static final double SHOOTER_ANGLE_MIN_DEG = 55.0;
    public static final double SHOOTER_ANGLE_MAX_DEG = 70.0;

    public static record ShooterFormula(double slope, double yIntercept, double min, double max) {
    }

    public static final ShooterFormula ANGLE_55 = new ShooterFormula(
            454.97,
            2033.9,
            Units.inchesToMeters(120),
            Units.inchesToMeters(200));
    public static final ShooterFormula ANGLE_65 = new ShooterFormula(
            448.4,
            2056.9,
            Units.inchesToMeters(120),
            Units.inchesToMeters(200));
}
