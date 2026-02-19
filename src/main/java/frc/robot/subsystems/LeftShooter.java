package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;

public class LeftShooter extends Shooter {
    private static LeftShooter instance;

    private LeftShooter() {
        super(ShooterConstants.LeftShooter.LEFT_MOTOR_CAN_ID, ShooterConstants.LeftShooter.RIGHT_MOTOR_CAN_ID, ShooterConstants.LeftShooter.TRANSLATION);
    }

    public static LeftShooter getInstance() {
        if (instance == null) {
            instance = new LeftShooter();
        }
        return instance;
    }
}
