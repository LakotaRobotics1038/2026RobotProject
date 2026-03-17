package frc.robot.constants;

public final class ClimbConstants {
    private ClimbConstants() {
    }

    public static final int MOTOR_CAN_ID = 15;
    // One over circumference of the pulley
    public static final double CLIMB_POSITION_CONVERSION_FACTOR = 1 / (0.75 * Math.PI);
    public static final double P = 0.8;
    public static final double I = 0.00005;
    public static final double D = 0.0;

    public static final double ZEROING_POWER = -0.3;

    public enum ClimbSetpoint {
        UP(40),
        DOWN(25),
        ZERO(0);

        private final double setpoint;

        ClimbSetpoint(double setpoint) {
            this.setpoint = setpoint;
        }

        public double getSetpoint() {
            return this.setpoint;
        }
    }
}