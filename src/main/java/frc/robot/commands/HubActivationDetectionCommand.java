package frc.robot.commands;

import java.util.function.DoubleConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class HubActivationDetectionCommand extends Command {
    public static final double SECONDS_BEFORE_HUB_ACTIVATION = 3;

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
                // The TELEOP shifts start at 130s remaining and repeat every 25s.
                double elapsed = 130.0 - matchTime;

                // floor-based modulus in doubles: elapsed - floor(elapsed / 25) * 25
                double timeSinceLast = elapsed - Math.floor(elapsed / 25.0) * 25.0;
                double timeToActivation = 25.0 - timeSinceLast;

                if (Math.floor(timeToActivation) == 0.0) {
                    timeToActivation = 0.0;
                }

                double rumblePower = 0.0;
                if (timeToActivation <= SECONDS_BEFORE_HUB_ACTIVATION) {
                    rumblePower = 1.0 - (timeToActivation / SECONDS_BEFORE_HUB_ACTIVATION);
                    rumblePower = MathUtil.clamp(rumblePower, 0.0, 1.0);
                }

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