package frc.robot.utils.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimpleDashboardValue<T> extends DashboardValue<T> {
    public SimpleDashboardValue(String name, T sendable) {
        super(sendable);
        SmartDashboard.putData(name, (Sendable) sendable);
    }

    @Override
    public void periodic() {
    }
}
