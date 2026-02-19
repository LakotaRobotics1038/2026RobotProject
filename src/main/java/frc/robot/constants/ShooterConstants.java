package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class ShooterConstants {
    private ShooterConstants() {
    }

    public static final int NEAR_LEFT_MOTOR_CAN_ID = 0;
    public static final int NEAR_RIGHT_MOTOR_CAN_ID = 0;
    public static final Translation2d NEAR_TRANSLATION = new Translation2d();

    public static final int FAR_LEFT_MOTOR_CAN_ID = 0;
    public static final int FAR_RIGHT_MOTOR_CAN_ID = 0;
    public static final Translation2d FAR_TRANSLATION = new Translation2d();

    public static final double RPM_TOLERANCE = 25.0;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;
}
