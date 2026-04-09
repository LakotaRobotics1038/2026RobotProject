package frc.robot.autons;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Dashboard;

public class AutonSelector {
    public enum AutonChoices {
        NoAuto,
        LeftTrenchAuto,
        RightTrenchAuto,
        LeftPreloadAuto,
        MiddlePreloadAuto,
        RightPreloadAuto,
        DepotAuto
    }

    // Choosers
    SendableChooser<AutonChoices> autoChooser;
    SendableChooser<Double> delayChooser;

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
        this.autoChooser = Dashboard.AUTO_CHOOSER.get();

        this.autoChooser.addOption("No Auto", AutonChoices.NoAuto);
        this.autoChooser.addOption("Left Trench Auto", AutonChoices.LeftTrenchAuto);
        this.autoChooser.addOption("Right Trench Auto", AutonChoices.RightTrenchAuto);
        this.autoChooser.addOption("Left Preload Only Auto", AutonChoices.LeftPreloadAuto);
        this.autoChooser.addOption("Middle Preload Only Auto", AutonChoices.MiddlePreloadAuto);
        this.autoChooser.addOption("Right Preload Only Auto", AutonChoices.RightPreloadAuto);
        this.autoChooser.setDefaultOption("Depot Auto", AutonChoices.DepotAuto);

        this.delayChooser = Dashboard.DELAY_CHOOSER.get();

        this.delayChooser.setDefaultOption("0 Seconds", 0.0);
        for (int i = 1; i <= 14; i++) {
            this.delayChooser.addOption(i + " Seconds", (double) i);
        }
    }

    public Auton chooseAuton() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        System.out.println(this.autoChooser.getSelected());
        try {
            switch (this.autoChooser.getSelected()) {
                case LeftTrenchAuto:
                    return new LeftTrenchAuto(alliance);
                case RightTrenchAuto:
                    return new RightTrenchAuto(alliance);
                case LeftPreloadAuto:
                    return new LeftPreloadAuto(alliance);
                case MiddlePreloadAuto:
                    return new MiddlePreloadAuto(alliance);
                case RightPreloadAuto:
                    return new RightPreloadAuto(alliance);
                case DepotAuto:
                    return new DepotAuto(alliance);
                default:
                    return null;
            }
        } catch (Exception e) {
            System.out.println("Choose Auton Failed " + e);
            return null;
        }
    }

    public double chooseDelay() {
        return this.delayChooser.getSelected();
    }
}