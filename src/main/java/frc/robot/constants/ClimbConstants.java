package frc.robot.constants;

public final class ClimbConstants {
    private ClimbConstants() {
    }

    public static final int MOTOR_CAN_ID = 0;
    public static final double MAX_CLIMB = 0;
    public static final double MIN_CLIMB = 0;
    // TODO Find the diameter of the winch pully and multiply it by pi.
    // One over the diameter of the winch pully
    public static final double CLIMB_POSITION_CONVERSION_FACTOR = 1 / (Math.PI);
    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public enum ClimbSetpoint {
        UP(MAX_CLIMB),
        DOWN(MIN_CLIMB),
        ZERO(0);

        private double setpoint;

        ClimbSetpoint(double setpoint){
            this.setpoint = setpoint;
        }

        public double getSetpoint() {
            return this.setpoint;
        }

    }
}