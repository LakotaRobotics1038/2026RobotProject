package frc.robot.constants;

public final class AcquisitionConstants {
    private AcquisitionConstants() {
    }

    public static final int PIVOT_MOTOR_CAN_ID = 4;
    public static final int INTAKE_MOTOR_CAN_ID = 3;

    public static final double PIVOT_P = 0.005;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;

    public static final double PIVOT_ENCODER_CONVERSION_FACTOR = 360 / 2;
    public static final double PIVOT_ALLOWED_ERROR_DEGREES = 2.0;
    public static final double PIVOT_POWER = 0.85;
    public static final double PIVOT_MAX_TRENCH_ANGLE_DEGREES = 50.0;
    public static final double PIVOT_MIN_ANGLE = 0;
    public static final double PIVOT_MAX_ANGLE = 105;

    public static final double INTAKE_ACQUIRE_DUTY_CYCLE = 1;
    public static final double INTAKE_DISPOSE_DUTY_CYCLE = -0.5;

    public enum AcquisitionSetpoint {
        RAISED(PIVOT_MAX_ANGLE),
        LOW_RAISE(30),
        HIGH_RAISE(55),
        LOWERED(PIVOT_MIN_ANGLE);

        private final double degrees;

        AcquisitionSetpoint(double degrees) {
            this.degrees = degrees;
        }

        public double getDegrees() {
            return degrees;
        }
    }
}
