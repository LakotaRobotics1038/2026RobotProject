package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.AcquisitionTrenchRetract;
import frc.robot.commands.RetractHoodsCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ZeroClimbCommand;
import frc.robot.constants.AcquisitionConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.libraries.XboxController1038;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.RectangleUtils;

public class OperatorJoystick extends XboxController1038 {
    public static OperatorJoystick instance;
    private final Dashboard dashboard = Dashboard.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();

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
                        () -> dashboard.nudgeManualShooterRPM(ShooterConstants.MANUAL_SHOOTER_RPM_STEP)));

        new Trigger(() -> this.getPOV().equals(PovPositions.Down))
                .onTrue(new InstantCommand(
                        () -> dashboard.nudgeManualShooterRPM(-ShooterConstants.MANUAL_SHOOTER_RPM_STEP)));

        new Trigger(() -> this.getPOV().equals(PovPositions.Left))
                .onTrue(new InstantCommand(dashboard::resetManualShooterRPM));

        new Trigger(() -> this.getPOV().equals(PovPositions.Right))
                .onTrue(new InstantCommand(dashboard::resetManualShooterRPM));

        this.start().onTrue(new ZeroClimbCommand());

        this.leftBumper().whileTrue(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.DISPOSE));
        this.rightBumper().whileTrue(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE));

        this.y().onTrue(new AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint.RAISED));
        this.a().onTrue(new AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint.LOWERED));
        this.x().whileTrue(new RetractHoodsCommand());

        new Trigger(this::isInTrench)
                .and(() -> DriverStation.isTeleopEnabled())
                .and(() -> !dashboard.isManualModeEnabled())
                .onTrue(new AcquisitionTrenchRetract());

        new Trigger(this::isInTrench)
                .and(() -> !dashboard.isManualModeEnabled())
                .whileTrue(new RetractHoodsCommand());

        this.rightTrigger().whileTrue(new ShootCommand(() -> this.b().getAsBoolean()));
    }

    private boolean isInTrench() {
        Translation2d robotPos = driveTrain.getState().Pose.getTranslation();
        return RectangleUtils.inRect(
                FieldConstants.TRENCH_RECTANGLES,
                robotPos);
    }
}
