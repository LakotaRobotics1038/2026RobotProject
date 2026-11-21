package frc.robot.commands;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.SwagLights;

public class HubActivationDetectionCommand extends Command {
    public static final double SECONDS_BEFORE_HUB_ACTIVATION = 3;
    private static final double TELEOP_DURATION_BEFORE_ENDGAME = 130;
    private static final double HUB_SWITCH_INTERVAL = 25;

    private final SwagLights swagLights = SwagLights.getInstance();
    private Boolean enabledFirst = null;
    private boolean hubEnablingSoon = false;
    private final DoubleConsumer hubActivationFeedback;

    public HubActivationDetectionCommand(DoubleConsumer hubActivationFeedback) {
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
                double elapsed = TELEOP_DURATION_BEFORE_ENDGAME - matchTime;
                double timeToActivation = elapsed % HUB_SWITCH_INTERVAL;
                double rumblePower = timeToActivation / SECONDS_BEFORE_HUB_ACTIVATION;
                hubActivationFeedback.accept(rumblePower);
                Dashboard.HUB_ACTIVATING.set(true);
                swagLights.setOperatorState(SwagLights.OperatorStates.HubActivating);
            } else if (wasEnablingSoon) {
                hubActivationFeedback.accept(0);
                Dashboard.HUB_ACTIVATING.set(false);
                swagLights.setOperatorState(SwagLights.OperatorStates.Default);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        hubActivationFeedback.accept(0);
    }

    private boolean isHubEnabledAt(double matchTime) {
        if (matchTime < 130 && matchTime > 30) {
            int elapsedInt = (int) (130 - matchTime);
            int interval = (elapsedInt / 25) % 2; // 0 for even intervals, 1 for odd
            boolean toggleHubEnabled = (interval == 0);
            return toggleHubEnabled == enabledFirst;
        }
        return true;
    }

    public boolean isHubEnablingSoon() {
        return hubEnablingSoon;
    }
}