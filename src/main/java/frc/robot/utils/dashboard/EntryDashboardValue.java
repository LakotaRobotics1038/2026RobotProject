package frc.robot.utils.dashboard;

import java.util.function.UnaryOperator;

public class EntryDashboardValue<T> extends DashboardValue<T> {
    private final UnaryOperator<T> transformer;

    public EntryDashboardValue(String name, UnaryOperator<T> transformer, T defaultValue) {
        super(name, defaultValue);
        this.transformer = transformer;
        this.entry.setDefaultValue(defaultValue);
    }

    @Override
    public void set(T value) {
        super.set(transformer.apply(value));
    }
}
