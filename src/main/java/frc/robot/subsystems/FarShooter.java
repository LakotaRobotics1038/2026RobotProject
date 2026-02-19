package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;

public class FarShooter extends Shooter {
    private static FarShooter instance;

    private FarShooter() {
        super(ShooterConstants.FAR_LEFT_MOTOR_CAN_ID, ShooterConstants.FAR_RIGHT_MOTOR_CAN_ID, ShooterConstants.FAR_TRANSLATION);
    }

    public static FarShooter getInstance() {
        if (instance == null) {
            instance = new FarShooter();
        }
        return instance;
    }
}
