package frc.robot.utils.dashboard;

import java.util.function.Consumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SendableDashboardValue<T> extends DashboardValue<T> {
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