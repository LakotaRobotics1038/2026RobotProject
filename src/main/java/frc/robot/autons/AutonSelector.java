package frc.robot.autons;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Dashboard;

public class AutonSelector {
    public enum AutonChoices {
        NoAuto,
        LeftTaxi,
        RightTaxi
    }

    // Choosers
    SendableChooser<AutonChoices> autoChooser;

    // Singleton Setup
    private static AutonSelector instance;

    public static AutonSelector getInstance() {
        if (instance == null) {
            System.out.println("Creating new AutonSelector");
            instance = new AutonSelector();
        }
        return instance;
    }

    private AutonSelector() {
        this.autoChooser = Dashboard.getInstance().getAutoChooser();

        this.autoChooser.setDefaultOption("No Auto", AutonChoices.NoAuto);
        this.autoChooser.addOption("Left Taxi", AutonChoices.LeftTaxi);
        this.autoChooser.addOption("Right Taxi", AutonChoices.RightTaxi);
    }

    public Auton chooseAuton() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        System.out.println(this.autoChooser.getSelected());
        try {
            switch (this.autoChooser.getSelected()) {
                case LeftTaxi:
                    return new LeftTaxi(alliance);
                case RightTaxi:
                    return new RightTaxi(alliance);
                default:
                    return null;
            }
        } catch (Exception e) {
            System.out.println("Choose Auton Failed " + e);
            return null;
        }
    }

    public double chooseDelay() {
        return Dashboard.getInstance().getDelay();
    }
}