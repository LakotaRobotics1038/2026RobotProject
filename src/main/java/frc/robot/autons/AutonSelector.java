package frc.robot.autons;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Dashboard;

public class AutonSelector {
    public enum AutonChoices {
        NoAuto,
        LeftAuto,
        SimpleLeftAuto
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
        this.autoChooser = Dashboard.getInstance().getAutoChooser();

        this.autoChooser.setDefaultOption("No Auto", AutonChoices.NoAuto);
        this.autoChooser.addOption("Left Auto", AutonChoices.LeftAuto);
        this.autoChooser.addOption("Left Auto Simple", AutonChoices.SimpleLeftAuto);


        this.delayChooser = Dashboard.getInstance().getDelayChooser();

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
                case LeftAuto:
                    return new LeftAuto(alliance);
                case SimpleLeftAuto:
                    return new SimpleLeftAuto(alliance);
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