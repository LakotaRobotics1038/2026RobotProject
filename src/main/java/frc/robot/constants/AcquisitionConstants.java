package frc.robot.constants;

public final class AcquisitionConstants {
    private AcquisitionConstants() {
    }

    public static final int PIVOT_MOTOR_CAN_ID = 4;
    public static final int INTAKE_MOTOR_CAN_ID = 3;

    public static final double PIVOT_P = 0.01;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;

    public static final double PIVOT_ENCODER_CONVERSION_FACTOR = 360 / 2;
    public static final double PIVOT_ALLOWED_ERROR_DEGREES = 2.0;
    public static final double PIVOT_POWER = 0.85;

    public static final double INTAKE_P = 0.01;
    public static final double INTAKE_I = 0.0;
    public static final double INTAKE_D = 0.0;

    public static final double INTAKE_S = 0.0;
    public static final double INTAKE_V = NeoMotorConstants.BATTERY_VOLTAGE / NeoMotorConstants.NEO_FREE_SPEED_RPM;
    public static final double INTAKE_A = 0.0;
    public static final double INTAKE_ACQUIRE_RPM = 4000;
    public static final double INTAKE_DISPOSE_RPM = -100;

    public enum AcquisitionSetpoint {
        RAISED(105),
        LOWERED(0);

        private final double degrees;

        AcquisitionSetpoint(double degrees) {
            this.degrees = degrees;
        }

        public double getDegrees() {
            return degrees;
        }
    }
}
