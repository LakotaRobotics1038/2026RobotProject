package frc.robot.utils.dashboard;

import java.util.function.UnaryOperator;

public class EntryDashboardValue<T> extends DashboardValue<T> {
    private final UnaryOperator<T> transformer;

    public EntryDashboardValue(String name, UnaryOperator<T> transformer, T defaultValue, boolean persistent) {
        super(name, defaultValue, persistent);
        this.transformer = transformer;
        this.entry.setDefaultValue(defaultValue);
    }

    public EntryDashboardValue(String name, UnaryOperator<T> transformer, T defaultValue) {
        this(name, transformer, defaultValue, false);
    }

    @Override
    public void set(T value) {
        super.set(transformer.apply(value));
    }
}
