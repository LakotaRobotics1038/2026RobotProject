package frc.robot.utils.dashboard;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuppliedDashboardValue<T> extends DashboardValue<T> {
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