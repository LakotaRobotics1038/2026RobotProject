package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.dashboard.DashboardValue;

public final class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_TOP_CAN_ID = 5;
    public static final int SHOOTER_MOTOR_LEFT_BOTTOM_CAN_ID = 3;
    public static final int SHOOTER_MOTOR_RIGHT_TOP_CAN_ID = 16;
    public static final int SHOOTER_MOTOR_RIGHT_BOTTOM_CAN_ID = 15;
    public static final Translation2d SHOOTER_BARREL_CENTER = new Translation2d(Units.inchesToMeters(5.202363), 0);

    public static final double OPERATING_TOLERANCE = 75;

    public static final double P = 0.000175;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = NeoMotorConstants.BATTERY_VOLTAGE / NeoMotorConstants.VORTEX_FREE_SPEED_RPM;
    public static final double A = 0.0;

    public static final double SHOOTER_DIRECTION_FROM_FORWARD_RAD = -Math.PI / 2.0;
    public static final double MANUAL_SHOOTER_RPM = 2900.0;
    public static final double MANUAL_SHOOTER_RPM_STEP = 50.0;
    public static final double MANUAL_SHOOTER_MIN_RPM = 2000.0;
    // Make sure there's a 0 at the end so manual mode goes by the shooter RPM step
    public static final double MANUAL_SHOOTER_MAX_RPM = (int) (NeoMotorConstants.VORTEX_FREE_SPEED_RPM
            / MANUAL_SHOOTER_RPM_STEP) * MANUAL_SHOOTER_RPM_STEP;

    /**
     * List of angles and their corresponding shooter formulas. The formula is used
     * to calculate the RPM of the shooter
     * based on the distance to the target. The min and max values represent the
     * range of that angle.
     */
    public static final List<ShooterConstants.ShooterFormula> SHOOTER_FORMULAS = List.of(
            new ShooterConstants.ShooterFormula(
                    68,
                    393.7,
                    2150,
                    1.524,
                    2.286),
            new ShooterConstants.ShooterFormula(
                    59,
                    397.69,
                    1949.3,
                    2.286,
                    10000.0));

    public static final class ShooterFormula {
        private final DashboardValue<Double> slope;
        private final DashboardValue<Double> yIntercept;
        private final double min;
        private final double max;
        private final double angle;

        public ShooterFormula(
                double angle,
                double shooterSlope,
                double shooterYIntercept,
                double min,
                double max) {
            this.angle = angle;
            this.slope = new DashboardValue<>("Shooter Slope " + (int) angle, shooterSlope);
            this.yIntercept = new DashboardValue<>("Shooter Y Intercept " + (int) angle, shooterYIntercept);
            this.min = min;
            this.max = max;
        }

        public double getAngle() {
            return angle;
        }

        public double getMin() {
            return min;
        }

        public double getMax() {
            return max;
        }

        public double getSlope() {
            return slope.get();
        }

        public void setSlope(double slope) {
            this.slope.set(slope);
        }

        public double getYIntercept() {
            return yIntercept.get();
        }

        public void setYIntercept(double yIntercept) {
            this.yIntercept.set(yIntercept);
        }

        public double getShooterRPM(double distance) {
            return getSlope() * distance + getYIntercept();
        }
    }
}
