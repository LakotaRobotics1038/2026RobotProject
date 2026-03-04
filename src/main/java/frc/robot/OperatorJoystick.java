package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.Climb;

public class OperatorJoystick extends XboxController1038 {
    private static final Climb climb = Climb.getInstance();

    public static OperatorJoystick instance = new OperatorJoystick();

    public static OperatorJoystick getInstance() {
        if (instance == null) {
            instance = new OperatorJoystick();
        }
        return instance;
    }

    private OperatorJoystick() {
        super(IOConstants.OPERATOR_CONTROLLER_PORT);

        this.leftBumper().onTrue(new InstantCommand(climb::climbDown));
        this.rightBumper().onTrue(new InstantCommand(climb::climbUp));
    }
}
