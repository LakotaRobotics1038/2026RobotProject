package frc.robot.constants;

public final class ShooterHoodsConstants {
    public static final int LEFT_MOTOR_CAN_ID = 0;
    public static final int RIGHT_MOTOR_CAN_ID = 0;

    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0;
    public static final double V = NeoMotorConstants.BATTERY_VOLTAGE / NeoMotorConstants.NEO_550_FREE_SPEED_RPM;
    public static final double A = 0;

    public static final double HOOD_ENCODER_CONVERSION_FACTOR = 360;

    public static final double SHOOTER_NO_RETRACTION_ANGLE = 54.0;
    public static final double SHOOTER_FULL_RETRACTION_ANGLE = 73.0;

    public static final double MANUAL_SHOOTER_DEFAULT_ANGLE = 59;
    public static final double MANUAL_SHOOTER_ANGLE_INCREMENT = 2;
}
