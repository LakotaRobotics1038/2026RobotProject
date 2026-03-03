package frc.robot;

import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;

public class OperatorJoystick extends XboxController1038 {
    public static OperatorJoystick instance = new OperatorJoystick();

    public static OperatorJoystick getInstance() {
        if (instance == null) {
            instance = new OperatorJoystick();
        }
        return instance;
    }

    private OperatorJoystick() {
        super(IOConstants.OPERATOR_CONTROLLER_PORT);
    }
}
