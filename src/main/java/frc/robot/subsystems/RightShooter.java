package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;
import frc.robot.utils.Shooter;

public class RightShooter extends Shooter {
    private static RightShooter instance;

    private RightShooter() {
        super(ShooterConstants.RightShooter.LEFT_SHOOTER_CAN_ID, ShooterConstants.RightShooter.RIGHT_SHOOTER_CAN_ID);
    }

    public static RightShooter getInstance() {
        if (instance == null) {
            instance = new RightShooter();
        }
        return instance;
    }
}
