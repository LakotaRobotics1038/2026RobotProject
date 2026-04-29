package frc.robot.utils.dashboard;

import java.util.function.Consumer;

import edu.wpi.first.util.sendable.Sendable;

public class SendableDashboardValue<T extends Sendable> extends DashboardValue<T> {
    private final Consumer<T> consumer;

    public SendableDashboardValue(String name, Consumer<T> consumer, T defaultValue, boolean persistent) {
        super(name, defaultValue, persistent);
        this.consumer = consumer;
    }

    public SendableDashboardValue(String name, Consumer<T> consumer, T defaultValue) {
        this(name, consumer, defaultValue, false);
    }

    @Override
    public void periodic() {
        consumer.accept(get());
    }
}
