package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autons.AutonSelector.AutonChoices;
import frc.robot.constants.AcquisitionPivotConstants;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterHoodsConstants;

public class Dashboard extends SubsystemBase {
    // Choosers
    private final SendableChooser<AutonChoices> autoChooser = new SendableChooser<>();
    private final SendableChooser<Double> delayChooser = new SendableChooser<>();

    // Singleton Setup
    private static Dashboard instance;

    public static Dashboard getInstance() {
        if (instance == null) {
            System.out.println("Creating a new Dashboard");
            instance = new Dashboard();
        }
        return instance;
    }

    private Dashboard() {
        SmartDashboard.putData(DashboardConstants.AUTON_CHOICES, autoChooser);
        SmartDashboard.putData(DashboardConstants.DELAY_CHOICES, delayChooser);

        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            SmartDashboard.putNumber(DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope());
            SmartDashboard.putNumber(DashboardConstants.shooterYInterceptKey(formula.getAngle()),
                    formula.getYIntercept());
        }

        Field2d field = (Field2d) DashboardValue.FIELD.get();
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> field.getObject("target pose").setPose(pose));
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("poses").setPoses(poses));
    }

    @Override
    public void periodic() {
        for (DashboardValue value : DashboardValue.values()) {
            value.periodic();
        }

        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            formula.setSlope(SmartDashboard.getNumber(
                    DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope()));
            formula.setYIntercept(SmartDashboard.getNumber(
                    DashboardConstants.shooterYInterceptKey(formula.getAngle()), formula.getYIntercept()));
        }
    }

    /**
     * Removes the trajectory line from the dashboard
     */
    public void clearTrajectory() {
        ((Field2d) DashboardValue.FIELD.get()).getObject("traj").setPoses(new ArrayList<>());
    }

    /**
     * Gets the sendable chooser for Auton Modes
     *
     * @return The sendable chooser
     */
    public SendableChooser<AutonChoices> getAutoChooser() {
        return autoChooser;
    }

    public SendableChooser<Double> getDelayChooser() {
        return delayChooser;
    }

    public void setHubAligned(boolean hubAligned) {
        DashboardValue.HUB_ALIGNED.set(hubAligned);
    }

    public boolean isManualModeEnabled() {
        return (boolean) DashboardValue.MANUAL_MODE_ENABLED.get();
    }

    public double getManualShooterRPM() {
        return (double) DashboardValue.MANUAL_SHOOTER_RPM.get();
    }

    public double getManualShooterHoodAngle() {
        return (double) DashboardValue.MANUAL_SHOOTER_HOOD_ANGLE.get();
    }

    public void nudgeManualShooterRPMBackward() {
        DashboardValue.MANUAL_SHOOTER_RPM.set(getManualShooterRPM() - ShooterConstants.MANUAL_SHOOTER_RPM_STEP);
    }

    public void nudgeManualShooterRPMForward() {
        DashboardValue.MANUAL_SHOOTER_RPM.set(getManualShooterRPM() + ShooterConstants.MANUAL_SHOOTER_RPM_STEP);
    }

    public void resetManualShooterRPM() {
        DashboardValue.MANUAL_SHOOTER_RPM.set(ShooterConstants.MANUAL_SHOOTER_RPM);
    }

    public void nudgeManualShooterHoodAngleBackward() {
        DashboardValue.MANUAL_SHOOTER_HOOD_ANGLE.set(
                getManualShooterHoodAngle() - ShooterHoodsConstants.MANUAL_SHOOTER_ANGLE_INCREMENT);
    }

    public void nudgeManualShooterHoodAngleForward() {
        DashboardValue.MANUAL_SHOOTER_HOOD_ANGLE.set(
                getManualShooterHoodAngle() + ShooterHoodsConstants.MANUAL_SHOOTER_ANGLE_INCREMENT);
    }

    public void resetManualShooterHoodAngle() {
        DashboardValue.MANUAL_SHOOTER_HOOD_ANGLE.set(ShooterHoodsConstants.MANUAL_SHOOTER_DEFAULT_ANGLE);
    }

    public double getAcquisitionMinWiggle() {
        return (double) DashboardValue.ACQUISITION_MIN_WIGGLE.get();
    }

    public double getAcquisitionMaxWiggle() {
        return (double) DashboardValue.ACQUISITION_MAX_WIGGLE.get();
    }

    public enum DashboardValue {
        ROBOT_X(DashboardConstants.ROBOT_X,
                () -> DriveTrain.getInstance().getX(), 0.0),
        ROBOT_Y(DashboardConstants.ROBOT_Y,
                () -> DriveTrain.getInstance().getY(), 0.0),
        ROBOT_ROT(DashboardConstants.ROBOT_ROT,
                () -> DriveTrain.getInstance().getRotation(), 0.0),
        HUB_ALIGNED(DashboardConstants.HUB_ALIGNED, false),
        MANUAL_MODE_ENABLED(DashboardConstants.MANUAL_MODE_ENABLED, false),
        MANUAL_SHOOTER_RPM(DashboardConstants.MANUAL_SHOOTER_RPM,
                v -> MathUtil.clamp((double) v,
                        ShooterConstants.MANUAL_SHOOTER_MIN_RPM,
                        ShooterConstants.MANUAL_SHOOTER_MAX_RPM),
                ShooterConstants.MANUAL_SHOOTER_RPM),
        MANUAL_SHOOTER_HOOD_ANGLE(DashboardConstants.MANUAL_SHOOTER_HOOD_ANGLE,
                v -> MathUtil.clamp((double) v,
                        ShooterHoodsConstants.SHOOTER_NO_RETRACTION_ANGLE,
                        ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE),
                ShooterHoodsConstants.MANUAL_SHOOTER_DEFAULT_ANGLE),
        ACQUISITION_MIN_WIGGLE(DashboardConstants.ACQUISITION_MIN_WIGGLE,
                AcquisitionPivotConstants.MIN_WIGGLE),
        ACQUISITION_MAX_WIGGLE(DashboardConstants.ACQUISITION_MAX_WIGGLE,
                AcquisitionPivotConstants.MAX_WIGGLE),
        FIELD(new Field2d(),
                v -> ((Field2d) v).setRobotPose(
                        DriveTrain.getInstance().getState().Pose));

        private final Supplier<Object> supplier;
        private final UnaryOperator<Object> transformer;
        private final Consumer<Object> periodicAction;
        private final NetworkTableEntry entry;
        private Object value;

        DashboardValue(String name, Supplier<Object> supplier, Object defaultValue) {
            this.supplier = supplier;
            this.transformer = null;
            this.periodicAction = null;
            this.value = defaultValue;
            this.entry = SmartDashboard.getEntry(name);
            this.entry.setDefaultValue(defaultValue);
        }

        DashboardValue(String name, Object defaultValue) {
            this.supplier = null;
            this.transformer = null;
            this.periodicAction = null;
            this.value = defaultValue;
            this.entry = SmartDashboard.getEntry(name);
            this.entry.setDefaultValue(defaultValue);
        }

        DashboardValue(String name, UnaryOperator<Object> transformer, Object defaultValue) {
            this.supplier = null;
            this.transformer = transformer;
            this.periodicAction = null;
            this.value = defaultValue;
            this.entry = SmartDashboard.getEntry(name);
            this.entry.setDefaultValue(defaultValue);
        }

        DashboardValue(Sendable sendable, Consumer<Object> periodicAction) {
            this.supplier = null;
            this.transformer = null;
            this.periodicAction = periodicAction;
            this.value = sendable;
            this.entry = null;
            SmartDashboard.putData(sendable);
        }

        public Object get() {
            return supplier != null ? supplier.get() : value;
        }

        public void set(Object newValue) {
            this.value = transformer != null ? transformer.apply(newValue) : newValue;
            if (entry != null) {
                entry.setValue(this.value);
            }
        }

        private void periodic() {
            if (periodicAction != null) {
                periodicAction.accept(this.value);
            } else if (supplier != null) {
                entry.setValue(supplier.get());
            } else {
                Object dashValue = entry.getValue().getValue();
                this.value = transformer != null ? transformer.apply(dashValue) : dashValue;
                entry.setValue(this.value);
            }
        }
    }
}
