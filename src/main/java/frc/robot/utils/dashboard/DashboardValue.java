package frc.robot.utils.dashboard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardValue<T> {
    private static final List<DashboardValue<?>> allValues = new ArrayList<>();

    protected final NetworkTableEntry entry;

    public DashboardValue(String name, T defaultValue) {
        this.entry = SmartDashboard.getEntry(name);
        this.entry.setDefaultValue(defaultValue);
        allValues.add(this);
    }

    public static List<DashboardValue<?>> values() {
        return Collections.unmodifiableList(allValues);
    }

    @SuppressWarnings("unchecked")
    public T get() {
        return (T) entry.getValue().getValue();
    }

    public void set(T newValue) {
        entry.setValue(newValue);
    }

    public void periodic() {
    }
}
