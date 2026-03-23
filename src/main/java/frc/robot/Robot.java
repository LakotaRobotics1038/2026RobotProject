// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.ControlWord;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autons.Auton;
import frc.robot.autons.AutonSelector;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwagLights.LEDState;

public class Robot extends TimedRobot {
    // Singleton Instances
    private final AutonSelector autonSelector = AutonSelector.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();

    // Variables
    private Auton autonomousCommand;
    private final ControlWord controlWordCache = new ControlWord();

    // Subsystems
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Vision vision = Vision.getInstance();

    // Human Interface Devices

    @Override
    public void robotInit() {
        // Singleton instances that need to be created but not referenced
        DriverJoystick.getInstance();
        OperatorJoystick.getInstance();
        Dashboard.getInstance();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        addPeriodic(swagLights::periodic, 0.25);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        vision.frontCamGetEstimatedGlobalPose()
                .ifPresent(estimatedPose -> driveTrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                        estimatedPose.timestampSeconds,
                        vision.getFrontEstimationStdDevs()));

        vision.backCamGetEstimatedGlobalPose()
                .ifPresent(estimatedPose -> driveTrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                        estimatedPose.timestampSeconds,
                        vision.getBackEstimationStdDevs()));
    }

    @Override
    public void disabledInit() {
        System.out.println("Robot Disabled");
        DriverStationJNI.getControlWord(controlWordCache);
        LEDState.ENABLED.setActive(false);
        if (controlWordCache.getEStop()) {
            LEDState.EMERGENCY_STOP.setActive(true);
            LEDState.DISABLED.setActive(false);
        } else {
            LEDState.EMERGENCY_STOP.setActive(false);
            LEDState.DISABLED.setActive(true);
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        LEDState.DISABLED.setActive(false);
        LEDState.ENABLED.setActive(true);
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = autonSelector.chooseAuton();

        if (autonomousCommand != null) {
            Pose2d initialPose = autonomousCommand.getInitialPose();
            if (initialPose != null) {
                driveTrain.resetPose(initialPose);
            }
            driveTrain.configNeutralMode(SwerveConstants.AUTON_DRIVING_MOTOR_NEUTRAL_MODE);
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        Dashboard.getInstance().clearTrajectory();
        driveTrain.configNeutralMode(SwerveConstants.TELEOP_DRIVING_MOTOR_NEUTRAL_MODE);
        // CommandScheduler.getInstance().schedule(new
        // FieldManagementCommand(rumblePower -> {
        // DriverJoystick.getInstance().setRumble(rumblePower);
        // OperatorJoystick.getInstance().setRumble(rumblePower);
        // }));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        driveTrain.setX();
        // vision.stopRecording();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
