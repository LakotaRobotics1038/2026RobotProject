package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;

public class OperatorJoystick extends XboxController1038 {
    public static OperatorJoystick instance;

    public static OperatorJoystick getInstance() {
        if (instance == null) {
            instance = new OperatorJoystick();
        }
        return instance;
    }

    private OperatorJoystick() {
        super(IOConstants.OPERATOR_CONTROLLER_PORT);

        new Trigger(() -> this.getPOV().equals(PovPositions.Up))
                .onTrue(new ClimbCommand(ClimbConstants.ClimbSetpoint.UP));
        new Trigger(() -> this.getPOV().equals(PovPositions.Down))
                .onTrue(new ClimbCommand(ClimbConstants.ClimbSetpoint.DOWN));

        this.leftBumper().whileTrue(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.DISPOSE));
        this.rightBumper().whileTrue(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE));

        this.y().onTrue(new AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint.RAISED));
        this.a().onTrue(new AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint.LOWERED));
    }
}
