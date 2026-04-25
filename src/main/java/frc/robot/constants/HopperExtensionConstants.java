package frc.robot.constants;

public final class HopperExtensionConstants {
    public static final int MOTOR_CAN_ID = 4;

    public static final double P = 0.2;
    public static final double I = 0.001;
    public static final double D = 0;

    public static final double EXTENSION_TO_MOTOR_RATIO = 48.0 / 10;
    public static final double EXTENSION_GEARBOX = 9;
    public static final double TEETH_PER_INCH = 3.18;
    public static final double POSITION_CONVERSION_FACTOR = 1 / (HopperExtensionConstants.EXTENSION_TO_MOTOR_RATIO
            * HopperExtensionConstants.EXTENSION_GEARBOX);

    public static final double IN_DUTY_CYCLE = -0.5;
    public static final double IN_DUTY_CYCLE_WHILE_SHOOTING = -0.1;
}
