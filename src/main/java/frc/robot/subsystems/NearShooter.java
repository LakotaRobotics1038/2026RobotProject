package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import frc.robot.constants.ShooterConstants;

public class NearShooter extends Shooter {
    private static NearShooter instance;

    private NearShooter() {
        super(ShooterConstants.NEAR_LEFT_MOTOR_CAN_ID, ShooterConstants.NEAR_RIGHT_MOTOR_CAN_ID, ShooterConstants.NEAR_TRANSLATION);
    }

    public static NearShooter getInstance() {
        if (instance == null) {
            instance = new NearShooter();
        }
        return instance;
    }
}
