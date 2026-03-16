package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autons.Paths;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoAlignClimbCommand extends Command {
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private PathPlannerPath path;
    private Optional<RobotConfig> robotConfig;
    private Supplier<Pose2d> currentPose = () -> driveTrain.getState().Pose;
    private Supplier<ChassisSpeeds> currentSpeed = () -> driveTrain.getState().Speeds;
    private BiConsumer<ChassisSpeeds, DriveFeedforwards> output = (ChassisSpeeds speeds,
            DriveFeedforwards feedForwards) -> {
        driveTrain.setControl(
                new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedForwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedForwards.robotRelativeForcesYNewtons()));
    };
    private PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.P_X_CONTROLLER, AutoConstants.I_X_CONTROLLER,
                    AutoConstants.D_CONTROLLER),
            new PIDConstants(AutoConstants.P_THETA_CONTROLLER,
                    AutoConstants.I_THETA_CONTROLLER,
                    AutoConstants.D_THETA_CONTROLLER));

    private FollowPathCommand command;
    private Alliance alliance;

    public AutoAlignClimbCommand(Optional<Alliance> alliance) {
        Pose2d goal = currentPose.get().getY() > 4 ? FieldConstants.TOWER_DEPOT : FieldConstants.TOWER_OUTPOST;
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose.get(), goal);

        path = new PathPlannerPath(waypoints, Paths.getOutpostClimb().getGlobalConstraints(),
                new IdealStartingState(getMagnitude(), currentPose.get().getRotation()),
                new GoalEndState(0, goal.getRotation()));
        this.alliance = alliance.get();
        robotConfig = AutoConstants.ROBOT_CONFIG;
        addRequirements(driveTrain);
    }

    private double getMagnitude() {
        double magnitude = Math
                .sqrt(Math.pow(this.currentSpeed.get().vxMetersPerSecond, 2)
                        + Math.pow(this.currentSpeed.get().vyMetersPerSecond, 2));
        return magnitude;
    }

    @Override
    public void initialize() {
        command = new FollowPathCommand(path,
                currentPose,
                currentSpeed,
                output,
                driveController,
                robotConfig.get(),
                () -> alliance == Alliance.Red,
                driveTrain);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean isInterrupted) {
        command.end(isInterrupted);
    }
}
