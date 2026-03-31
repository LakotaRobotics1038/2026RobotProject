package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Dashboard;

public class AutonSelector {
    @FunctionalInterface
    private interface AutonFactory {
        Auton create(Optional<Alliance> alliance) throws IOException, ParseException;
    }

    public enum AutonChoice {
        NO_AUTO("No Auto", null),
        LEFT_AUTO("Left Auto", LeftAuto::new),
        LEFT_AUTO_SHOOT("Left Auto Shoot", LeftAutoShoot::new),
        MIDDLE_AUTO_SHOOT("Middle Auto Shoot", MiddleAutoShoot::new),
        RIGHT_AUTO_SHOOT("Right Auto Shoot", RightAutoShoot::new),
        LEFT_AUTO_DEPOT_SHOOT("Left Auto Depot Shoot", LeftAutoDepotShoot::new);

        private final String name;
        private final AutonFactory factory;

        AutonChoice(String name, AutonFactory factory) {
            this.name = name;
            this.factory = factory;
        }

        public String getName() {
            return name;
        }

        public Auton getAuton(Optional<Alliance> alliance) {
            try {
                return factory != null ? factory.create(alliance) : null;
            } catch (Exception e) {
                DriverStation.reportError("Choose Auton Failed " + e, true);
                return null;
            }
        }
    }

    // Choosers
    SendableChooser<AutonChoice> autoChooser;
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

        AutonChoice[] choices = AutonChoice.values();
        AutonChoice defaultChoice = AutonChoice.NO_AUTO;
        this.autoChooser.setDefaultOption(defaultChoice.getName(), defaultChoice);
        for (AutonChoice choice : choices) {
            if (choice != defaultChoice) {
                this.autoChooser.addOption(choice.getName(), choice);
            }
        }

        this.delayChooser = Dashboard.DELAY_CHOOSER.get();

        this.delayChooser.setDefaultOption("0 Seconds", 0.0);
        for (int i = 1; i <= 14; i++) {
            this.delayChooser.addOption(i + " Seconds", (double) i);
        }
    }

    public Auton chooseAuton() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        AutonChoice selectedChoice = this.autoChooser.getSelected();

        System.out.println(selectedChoice);

        if (selectedChoice == null) {
            DriverStation.reportError("Selected Auton was null", false);
            return null;
        }

        return selectedChoice.getAuton(alliance);
    }

    public double chooseDelay() {
        return this.delayChooser.getSelected();
    }
}