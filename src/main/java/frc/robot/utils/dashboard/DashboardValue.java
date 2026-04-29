package frc.robot.utils.dashboard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardValue<T> {
    private static final List<DashboardValue<?>> allValues = new ArrayList<>();

    protected final String name;
    protected final NetworkTableEntry entry;
    protected final boolean isSendable;

    public DashboardValue(String name, T defaultValue, boolean persistent) {
        this.name = name;
        this.entry = SmartDashboard.getEntry(name);
        this.isSendable = defaultValue instanceof Sendable;
        if (isSendable) {
            SmartDashboard.putData(name, (Sendable) defaultValue);
            if (persistent) {
                SmartDashboard.setPersistent(name);
            }
        } else {
            this.entry.setDefaultValue(defaultValue);
            if (persistent) {
                this.entry.setPersistent();
            }
        }
        allValues.add(this);
    }

    public DashboardValue(String name, T defaultValue) {
        this(name, defaultValue, false);
    }

    public static List<DashboardValue<?>> values() {
        return new ArrayList<>(allValues);
    }

    @SuppressWarnings("unchecked")
    public T get() {
        if (isSendable) {
            return (T) SmartDashboard.getData(name);
        }
        return (T) entry.getValue().getValue();
    }

    public void set(T value) {
        if (value instanceof Sendable) {
            SmartDashboard.putData(name, (Sendable) value);
        } else {
            entry.setValue(value);
        }
    }

    public void periodic() {
    }
}
