package frc.robot.constants;

public class AcquisitionConstants {
    private AcquisitionConstants() {
    }

    public static final int PIVOT_CAN_ID = 0;
    public static final int INTAKE_CAN_ID = 0;

    public static final double PIVOT_P = 0;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;

    public static final double PIVOT_S = 0;
    public static final double PIVOT_V = 0;
    public static final double PIVOT_A = 0;
    public static final double PIVOT_ALLOWED_ERROR_DEGREES = 2.0;

    public static final double INTAKE_P = 0.0;
    public static final double INTAKE_I = 0.0;
    public static final double INTAKE_D = 0.0;

    public static final double INTAKE_S = 0.0;
    public static final double INTAKE_V = 0.0;
    public static final double INTAKE_A = 0.0;

    public static final double RAISED_DEGREES = 0;
    public static final double LOWERED_DEGREES = 90;

    public static final double INTAKE_ACQUIRE_RPM = 4000;
    public static final double INTAKE_DISPOSE_RPM = -1000;

    public enum Setpoint {
        RAISED,
        LOWERED
    }
}
