package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.HopperExtensionCommand.ExtensionDirection;
import frc.robot.commands.IndexerCommand.IndexerDirection;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AcquisitionCommand.IntakeDirection;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.IOConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.Dashboard;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterHoodConstants;

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
                .onTrue(new InstantCommand(
                        () -> Dashboard.MANUAL_SHOOTER_RPM.set(
                                Dashboard.MANUAL_SHOOTER_RPM.get()
                                        + ShooterConstants.MANUAL_SHOOTER_RPM_STEP)));

        new Trigger(() -> this.getPOV().equals(PovPositions.Down))
                .onTrue(new InstantCommand(
                        () -> Dashboard.MANUAL_SHOOTER_RPM.set(
                                Dashboard.MANUAL_SHOOTER_RPM.get()
                                        - ShooterConstants.MANUAL_SHOOTER_RPM_STEP)));
        new Trigger(() -> this.getPOV().equals(PovPositions.Left))
                .onTrue(new InstantCommand(
                        () -> Dashboard.MANUAL_SHOOTER_HOOD_ANGLE.set(
                                Dashboard.MANUAL_SHOOTER_HOOD_ANGLE.get()
                                        - ShooterHoodConstants.MANUAL_SHOOTER_ANGLE_INCREMENT)));

        new Trigger(() -> this.getPOV().equals(PovPositions.Right))
                .onTrue(new InstantCommand(
                        () -> Dashboard.MANUAL_SHOOTER_HOOD_ANGLE.set(
                                Dashboard.MANUAL_SHOOTER_HOOD_ANGLE.get()
                                        + ShooterHoodConstants.MANUAL_SHOOTER_ANGLE_INCREMENT)));

        this.leftBumper().whileTrue(new AcquisitionCommand(IntakeDirection.DISPOSE));
        this.rightBumper().whileTrue(new AcquisitionCommand(IntakeDirection.INTAKE));

        this.y().onTrue(new HopperExtensionCommand(ExtensionDirection.IN));
        this.a().onTrue(new HopperExtensionCommand(ExtensionDirection.OUT));
        this.x().whileTrue(new KickerCommand());
        this.b().whileTrue(new FeederCommand()).whileTrue(new IndexerCommand(IndexerDirection.INTAKE));
        this.start().onTrue(new InstantCommand(() -> {
            Dashboard.MANUAL_SHOOTER_RPM.set(ShooterConstants.MANUAL_SHOOTER_RPM);
            Dashboard.MANUAL_SHOOTER_HOOD_ANGLE.set(
                    ShooterHoodConstants.MANUAL_SHOOTER_DEFAULT_ANGLE);
        }));

        this.rightTrigger().whileTrue(new ShootCommand());

        new Trigger(Dashboard.HUB_ALIGNING::get)
                .onTrue(new InstantCommand(() -> setRumble(AlignCommand.HUB_ALIGNMENT_RUMBLE_INTENSITY)))
                .onFalse(new InstantCommand(() -> setRumble(0)));
    }
}
