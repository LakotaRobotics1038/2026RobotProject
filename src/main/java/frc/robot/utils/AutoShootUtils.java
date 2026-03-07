package frc.robot.utils;

import java.util.List;

public final class AutoShootUtils {
    private AutoShootUtils() {
    }

    public static class AutoShootFormula {
        private final double shooterSlope;
        private final double shooterYIntercept;
        private final double kickerSlope;
        private final double kickerYIntercept;
        private final double min;
        private final double max;
        private final double angle;

        private AutoShootFormula(
                double angle,
                double shooterSlope,
                double shooterYIntercept,
                double kickerSlope,
                double kickerYIntercept,
                double min,
                double max) {
            this.angle = angle;
            this.shooterSlope = shooterSlope;
            this.shooterYIntercept = shooterYIntercept;
            this.kickerSlope = kickerSlope;
            this.kickerYIntercept = kickerYIntercept;
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

        public double getShooterRPM(double distance) {
            return shooterSlope * distance + shooterYIntercept;
        }

        public double getKickerRPM(double distance) {
            return kickerSlope * distance + kickerYIntercept;
        }
    }

    /**
     * List of angles and their corresponding shooter formulas. The formula is used
     * to calculate the RPM of the shooter
     * based on the distance to the target. The min and max values represent the
     * range of that angle.
     */
    public static final List<AutoShootFormula> AUTO_SHOOT_FORMULAS = List.of(
            new AutoShootFormula(
                    55.0,
                    454.97,
                    2033.9,
                    -547.33,
                    4856.8,
                    1.7,
                    3.25),
            new AutoShootFormula(
                    65.0,
                    448.4,
                    2056.9,
                    687.67,
                    862.02,
                    2.5,
                    5.1)
    );
}
