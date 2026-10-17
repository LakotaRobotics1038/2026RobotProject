package frc.robot.utils.dashboard;

import java.util.function.Supplier;

public class SuppliedDashboardValue<T> extends DashboardValue<T> {
    private final Supplier<T> supplier;

    public SuppliedDashboardValue(String name, Supplier<T> supplier, T defaultValue, boolean persistent) {
        super(name, defaultValue, persistent);
        this.supplier = supplier;
    }

    public SuppliedDashboardValue(String name, Supplier<T> supplier, T defaultValue) {
        this(name, supplier, defaultValue, false);
    }

    @Override
    public void periodic() {
        set(supplier.get());
    }
}
