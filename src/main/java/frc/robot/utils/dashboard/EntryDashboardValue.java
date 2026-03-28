package frc.robot.utils.dashboard;

import java.util.function.UnaryOperator;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EntryDashboardValue<T> extends DashboardValue<T> {
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