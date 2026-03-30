package frc.robot.constants;

public final class AcquisitionPivotConstants {
    public static final int MOTOR_CAN_ID = 4;

    public static final double P = 0.005;
    public static final double I = 0;
    public static final double D = 0;

    public static final double ENCODER_CONVERSION_FACTOR = 360 / 2;
    public static final double ALLOWED_ERROR_DEGREES = 2.0;
    public static final double OPERATING_TOLERANCE = 5.0;
    public static final double POWER = 0.85;
    public static final double MAX_TRENCH_ANGLE_DEGREES = 50.0;
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 105;
    public static final double MIN_WIGGLE = 30;
    public static final double MAX_WIGGLE = 75;

    public enum PivotSetpoint {
        RAISED(MAX_ANGLE),
        LOW_RAISE(30),
        HIGH_RAISE(55),
        LOWERED(MIN_ANGLE);

        private final double degrees;

        PivotSetpoint(double degrees) {
            this.degrees = degrees;
        }

        public double getDegrees() {
            return degrees;
        }
    }
}
