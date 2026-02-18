package frc.robot.constants;

public final class ShooterConstants {
    private ShooterConstants() {
    }

    public final class LeftShooter {
        public static final int LEFT_SHOOTER_CAN_ID = 0;
        public static final int RIGHT_SHOOTER_CAN_ID = 0;
    }

    public final class RightShooter {
        public static final int LEFT_SHOOTER_CAN_ID = 0;
        public static final int RIGHT_SHOOTER_CAN_ID = 0;
    }

    public static final double RPM_TOLERANCE = 25.0;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;
}
