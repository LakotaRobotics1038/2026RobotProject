package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;
import frc.robot.utils.Shooter;

public class LeftShooter extends Shooter {
    private static LeftShooter instance;

    private LeftShooter() {
        super(ShooterConstants.LeftShooter.LEFT_SHOOTER_CAN_ID, ShooterConstants.LeftShooter.RIGHT_SHOOTER_CAN_ID);
    }

    public static LeftShooter getInstance() {
        if (instance == null) {
            instance = new LeftShooter();
        }
        return instance;
    }
}
