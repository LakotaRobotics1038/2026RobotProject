package frc.robot.commands;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class FieldManagementCommand extends Command {
    public static final double SECONDS_BEFORE_HUB_ACTIVATION = 3;

    private Boolean enabledFirst = null;
    private boolean hubEnablingSoon = false;
    private final DoubleConsumer hubActivationFeedback;

    public FieldManagementCommand(DoubleConsumer hubActivationFeedback) {
        this.hubActivationFeedback = hubActivationFeedback;
    }

    @Override
    public void execute() {
        if (enabledFirst == null) {
            String gameData = DriverStation.getGameSpecificMessage();
            if (!gameData.isEmpty()) {
                Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                switch (gameData.charAt(0)) {
                    case 'R' -> enabledFirst = alliance != Alliance.Red;
                    case 'B' -> enabledFirst = alliance != Alliance.Blue;
                }
            }
        } else {
            double matchTime = DriverStation.getMatchTime();
            boolean currentlyEnabled = isHubEnabledAt(matchTime);
            boolean enabledSoon = isHubEnabledAt(matchTime - SECONDS_BEFORE_HUB_ACTIVATION);
            boolean wasEnablingSoon = hubEnablingSoon;
            hubEnablingSoon = !currentlyEnabled && enabledSoon;
            if (hubEnablingSoon) {
                double elapsed = 130 - matchTime;
                double timeToActivation = 25 - (elapsed % 25);
                double rumblePower = 1.0 - (timeToActivation / SECONDS_BEFORE_HUB_ACTIVATION);
                hubActivationFeedback.accept(rumblePower);
            } else if (wasEnablingSoon) {
                hubActivationFeedback.accept(0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        hubActivationFeedback.accept(0);
    }

    private boolean isHubEnabledAt(double matchTime) {
        if (matchTime < 130 && matchTime > 30) {
            boolean toggleHubEnabled = (int) (130 - matchTime) / 25 % 2 == 0;
            return toggleHubEnabled == enabledFirst;
        }
        return true;
    }

    public boolean isHubEnablingSoon() {
        return hubEnablingSoon;
    }
}
