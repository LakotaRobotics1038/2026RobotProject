package frc.robot.subsystems;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.dashboard.DashboardValue;

public class Dashboard extends SubsystemBase {
    // Singleton Setup
    private static Dashboard instance;

    public static Dashboard getInstance() {
        if (instance == null) {
            System.out.println("Creating a new Dashboard");
            instance = new Dashboard();
        }
        return instance;
    }

    private Dashboard() {
        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            SmartDashboard.putNumber(DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope());
            SmartDashboard.putNumber(DashboardConstants.shooterYInterceptKey(formula.getAngle()),
                    formula.getYIntercept());
        }

        Field2d field = DashboardValue.FIELD.get();
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> field.getObject("target pose").setPose(pose));
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("poses").setPoses(poses));
    }

    @Override
    public void periodic() {
        for (DashboardValue<?> value : DashboardValue.values()) {
            value.periodic();
        }

        for (ShooterConstants.ShooterFormula formula : ShooterConstants.SHOOTER_FORMULAS) {
            formula.setSlope(SmartDashboard.getNumber(
                    DashboardConstants.shooterSlopeKey(formula.getAngle()), formula.getSlope()));
            formula.setYIntercept(SmartDashboard.getNumber(
                    DashboardConstants.shooterYInterceptKey(formula.getAngle()), formula.getYIntercept()));
        }
    }
}
