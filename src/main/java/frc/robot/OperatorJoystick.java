package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.constants.ClimbConstants;
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

        this.leftBumper().onTrue(new ClimbCommand(ClimbConstants.ClimbSetpoint.DOWN));
        this.rightBumper().onTrue(new ClimbCommand(ClimbConstants.ClimbSetpoint.UP));
    }
}
