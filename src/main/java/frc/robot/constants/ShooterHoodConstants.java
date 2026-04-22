package frc.robot.constants;

public final class ShooterHoodConstants {
    public static final int LEFT_MOTOR_CAN_ID = 13;
    public static final int RIGHT_MOTOR_CAN_ID = 12;

    public static final double P = 16;
    public static final double I = 0.005;
    public static final double D = 0;

    public static final double SHOOTER_NO_RETRACTION_ANGLE = 50.0;
    public static final double SHOOTER_FULL_RETRACTION_ANGLE = 76.0;

    public static final double HOOD_ENCODER_CONVERSION_FACTOR = (SHOOTER_FULL_RETRACTION_ANGLE
            - SHOOTER_NO_RETRACTION_ANGLE) * 2;

    public static final double MANUAL_SHOOTER_DEFAULT_ANGLE = 55;
    public static final double MANUAL_SHOOTER_ANGLE_INCREMENT = 2;

    public static final double ANGLE_TOLERANCE = 0.5;
}
