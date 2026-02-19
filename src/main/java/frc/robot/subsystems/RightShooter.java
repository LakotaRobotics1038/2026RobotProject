package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;

public class RightShooter extends Shooter {
    private static RightShooter instance;

    private RightShooter() {
        super(ShooterConstants.RightShooter.LEFT_MOTOR_CAN_ID, ShooterConstants.RightShooter.RIGHT_MOTOR_CAN_ID, ShooterConstants.RightShooter.TRANSLATION);
    }

    public static RightShooter getInstance() {
        if (instance == null) {
            instance = new RightShooter();
        }
        return instance;
    }
}
