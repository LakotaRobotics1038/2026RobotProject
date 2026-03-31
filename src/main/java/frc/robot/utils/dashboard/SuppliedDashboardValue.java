package frc.robot.utils.dashboard;

import java.util.function.Supplier;

public class SuppliedDashboardValue<T> extends DashboardValue<T> {
    private final Supplier<T> supplier;

    public SuppliedDashboardValue(String name, Supplier<T> supplier, T defaultValue) {
        super(name, defaultValue);
        this.supplier = supplier;
    }

    @Override
    public void periodic() {
        set(supplier.get());
    }
}
