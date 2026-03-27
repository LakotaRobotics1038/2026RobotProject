package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractHoodsCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.Dashboard;
import frc.robot.utils.Direction;

public class OperatorJoystick extends XboxController1038 {
    public static OperatorJoystick instance;
    private final Dashboard dashboard = Dashboard.getInstance();

    public static OperatorJoystick getInstance() {
        if (instance == null) {
            instance = new OperatorJoystick();
        }
        return instance;
    }

    private OperatorJoystick() {
        super(IOConstants.OPERATOR_CONTROLLER_PORT);

        new Trigger(() -> this.getPOV().equals(PovPositions.Up))
                .onTrue(new InstantCommand(
                        dashboard::nudgeManualShooterRPMForward));

        new Trigger(() -> this.getPOV().equals(PovPositions.Down))
                .onTrue(new InstantCommand(
                        dashboard::nudgeManualShooterRPMBackward));
        new Trigger(() -> this.getPOV().equals(PovPositions.Left))
                .onTrue(new InstantCommand(dashboard::nudgeManualShooterHoodAngleBackward));

        new Trigger(() -> this.getPOV().equals(PovPositions.Right))
                .onTrue(new InstantCommand(dashboard::nudgeManualShooterHoodAngleForward));

        this.leftBumper().whileTrue(new IntakeCommand(Direction.FORWARD));
        this.rightBumper().whileTrue(new IntakeCommand(Direction.REVERSE));

        this.y().onTrue(new ExtensionCommand(Direction.FORWARD));
        this.a().onTrue(new ExtensionCommand(Direction.REVERSE));
        this.x().whileTrue(new RetractHoodsCommand());
        this.start().onTrue(new InstantCommand(() -> {
            dashboard.resetManualShooterRPM();
            dashboard.resetManualShooterHoodAngle();
        }));

        this.rightTrigger().whileTrue(new ShootCommand());
    }
}
