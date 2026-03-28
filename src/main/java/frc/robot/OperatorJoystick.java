package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.RetractHoodsCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.AcquisitionRunCommand.Mode;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.PrototypeAcq;

public class OperatorJoystick extends XboxController1038 {
    public static OperatorJoystick instance;
    private final Dashboard dashboard = Dashboard.getInstance();
    private final PrototypeAcq prototypeAcq = PrototypeAcq.getInstance();

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

        this.leftBumper().whileTrue(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.DISPOSE));
        this.rightBumper().whileTrue(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE));

        this.y().onTrue(new AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint.RAISED));
        this.a().onTrue(new AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint.LOWERED));
        this.x().whileTrue(new RetractHoodsCommand());
        this.b().onTrue(new InstantCommand(() -> prototypeAcq.start()))
                .onFalse(new InstantCommand(() -> prototypeAcq.stop()));
        this.start().onTrue(new InstantCommand(() -> {
            dashboard.resetManualShooterRPM();
            dashboard.resetManualShooterHoodAngle();
        }));

        this.rightTrigger().whileTrue(new ShootCommand(() -> this.b().getAsBoolean()));
    }
}
