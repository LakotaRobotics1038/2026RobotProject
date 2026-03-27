package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.ShooterConstants;

public class Dashboard extends SubsystemBase {
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
        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            SmartDashboard.putNumber(DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope());
            SmartDashboard.putNumber(DashboardConstants.shooterYInterceptKey(formula.getAngle()),
                    formula.getYIntercept());
        }

        Field2d field = DashboardConstants.FIELD.get();
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> field.getObject("target pose").setPose(pose));
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("poses").setPoses(poses));
    }

    @Override
    public void periodic() {
        for (DashboardValue<?> value : DashboardValue.values()) {
            value.periodic();
        }

        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            formula.setSlope(SmartDashboard.getNumber(
                    DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope()));
            formula.setYIntercept(SmartDashboard.getNumber(
                    DashboardConstants.shooterYInterceptKey(formula.getAngle()), formula.getYIntercept()));
        }
    }

    public abstract static class DashboardValue<T> {
        private static final ArrayList<DashboardValue<?>> allValues = new ArrayList<>();

        protected T value;

        protected DashboardValue(T defaultValue) {
            this.value = defaultValue;
            allValues.add(this);
        }

        public static ArrayList<DashboardValue<?>> values() {
            return allValues;
        }

        public T get() {
            return value;
        }

        public void set(T newValue) {
            this.value = newValue;
        }

        public abstract void periodic();
    }

    public static class SuppliedDashboardValue<T> extends DashboardValue<T> {
        private final Supplier<T> supplier;
        private final NetworkTableEntry entry;

        public SuppliedDashboardValue(String name, Supplier<T> supplier, T defaultValue) {
            super(defaultValue);
            this.supplier = supplier;
            this.entry = SmartDashboard.getEntry(name);
            this.entry.setDefaultValue(defaultValue);
        }

        @Override
        public T get() {
            return supplier.get();
        }

        @Override
        public void periodic() {
            entry.setValue(supplier.get());
        }
    }

    public static class EntryDashboardValue<T> extends DashboardValue<T> {
        private final UnaryOperator<T> transformer;
        private final NetworkTableEntry entry;

        public EntryDashboardValue(String name, T defaultValue) {
            this(name, null, defaultValue);
        }

        public EntryDashboardValue(String name, UnaryOperator<T> transformer, T defaultValue) {
            super(defaultValue);
            this.transformer = transformer;
            this.entry = SmartDashboard.getEntry(name);
            this.entry.setDefaultValue(defaultValue);
        }

        @Override
        public void set(T newValue) {
            this.value = transformer != null ? transformer.apply(newValue) : newValue;
            entry.setValue(this.value);
        }

        @SuppressWarnings("unchecked")
        @Override
        public void periodic() {
            T dashValue = (T) entry.getValue().getValue();
            this.value = transformer != null ? transformer.apply(dashValue) : dashValue;
            entry.setValue(this.value);
        }
    }

    public static class SimpleDashboardValue<T> extends DashboardValue<T> {
        public SimpleDashboardValue(String name, T sendable) {
            super(sendable);
            SmartDashboard.putData(name, (Sendable) sendable);
        }

        @Override
        public void periodic() {
        }
    }

    public static class SendableDashboardValue<T> extends DashboardValue<T> {
        private final Consumer<T> periodicAction;

        public SendableDashboardValue(T sendable, Consumer<T> periodicAction) {
            super(sendable);
            this.periodicAction = periodicAction;
            SmartDashboard.putData((Sendable) sendable);
        }

        @Override
        public void periodic() {
            periodicAction.accept(this.value);
        }
    }
}
