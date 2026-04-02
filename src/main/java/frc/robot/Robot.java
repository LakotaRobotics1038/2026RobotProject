// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.ControlWord;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autons.Auton;
import frc.robot.autons.AutonSelector;
import frc.robot.commands.HubActivationDetectionCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.SwagLights.RobotStates;
import frc.robot.subsystems.Vision;

public class Robot extends LoggedRobot {
    // Singleton Instances
    private final AutonSelector autonSelector = AutonSelector.getInstance();
    private final SwagLights swagLights = SwagLights.getInstance();

    // Variables
    private Auton autonomousCommand;
    private final ControlWord controlWordCache = new ControlWord();

    // Subsystems
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Vision vision = Vision.getInstance();

    private double swagLightsLastRunTime = 0;

    public Robot() {
        Logger.recordMetadata("ProjectName", "Wescavator");
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the
                                                          // user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a
                                                                                                  // new log
        }
    }

    @Override
    public void robotInit() {
        // Singleton instances that need to be created but not referenced
        DriverJoystick.getInstance();
        OperatorJoystick.getInstance();
        Dashboard.getInstance();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    @Override
    public void robotPeriodic() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - swagLightsLastRunTime >= 0.25) {
            swagLights.periodic();
            swagLightsLastRunTime = currentTime;
        }
        CommandScheduler.getInstance().run();

        vision.leftCamGetEstimatedGlobalPose()
                .ifPresent(estimatedPose -> driveTrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                        estimatedPose.timestampSeconds,
                        vision.getLeftEstimationStdDevs()));

        vision.backCamGetEstimatedGlobalPose()
                .ifPresent(estimatedPose -> driveTrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                        estimatedPose.timestampSeconds,
                        vision.getBackEstimationStdDevs()));
    }

    @Override
    public void disabledInit() {
        System.out.println("Robot Disabled");
        DriverStationJNI.getControlWord(controlWordCache);
        if (controlWordCache.getEStop()) {
            swagLights.setRobotState(RobotStates.EmergencyStop);
        } else {
            swagLights.setRobotState(RobotStates.Disabled);
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        swagLights.setRobotState(RobotStates.Enabled);
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
        Dashboard.getInstance().clearFieldTrajectory();
        driveTrain.configNeutralMode(SwerveConstants.TELEOP_DRIVING_MOTOR_NEUTRAL_MODE);
        CommandScheduler.getInstance().schedule(new HubActivationDetectionCommand(rumblePower -> {
            DriverJoystick.getInstance().setRumble(rumblePower);
            OperatorJoystick.getInstance().setRumble(rumblePower);
        }));
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
