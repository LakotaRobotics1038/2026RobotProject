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
        LEFT_AUTO_SHOOT("Left Auto Shoot", SimpleLeftAuto::new),
        MIDDLE_AUTO_SHOOT("Middle Auto Shoot", SimpleMiddleAuto::new),
        RIGHT_AUTO_SHOOT("Right Auto Shoot", SimpleRightAuto::new),
        LEFT_AUTO_DEPOT_SHOOT("Left Auto Depot Shoot", DepotLeftAuto::new);

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
            } catch (IOException | ParseException e) {
                System.out.println("Choose Auton Failed " + e);
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
        this.autoChooser = Dashboard.getInstance().getAutoChooser();

        AutonChoice[] choices = AutonChoice.values();
        this.autoChooser.setDefaultOption(choices[0].getName(), choices[0]);
        for (int i = 1; i < choices.length; i++) {
            this.autoChooser.addOption(choices[i].getName(), choices[i]);
        }

        this.delayChooser = Dashboard.getInstance().getDelayChooser();

        this.delayChooser.setDefaultOption("0 Seconds", 0.0);
        for (int i = 1; i <= 14; i++) {
            this.delayChooser.addOption(i + " Seconds", (double) i);
        }
    }

    public Auton chooseAuton() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        System.out.println(this.autoChooser.getSelected());
        return this.autoChooser.getSelected().getAuton(alliance);
    }

    public double chooseDelay() {
        return this.delayChooser.getSelected();
    }
}