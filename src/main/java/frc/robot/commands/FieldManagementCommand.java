package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class FieldManagementCommand extends Command {
    public static final double SECONDS_BEFORE_HUB_ACTIVATION = 5;

    private Boolean enabledFirst = null;
    private boolean hubEnablingSoon = false;

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
            hubEnablingSoon = !currentlyEnabled && enabledSoon;
        }
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
