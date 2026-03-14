package frc.robot.autons.depot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autons.Auton;
import frc.robot.autons.Paths;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public abstract class DepotNeutral extends Auton {
    public DepotNeutral(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);

        super.addCommands(
                // TODO 1 Empty Hopper
                followPathCommand(Paths.getDepotStartToShootingPath()),
                new AutoShootCommand().withDeadline(new WaitCommand(2))
        // TODO 2 Collect From Neutral
        // , followPathCommand(Paths.getDepotShootingToNeutralPath()),
        // new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)
        // .andThen(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE)
        // .raceWith(followPathCommand(Paths.getDepotAcquireFromNeutralPath())
        // .andThen(new WaitCommand(0.5))))
        // TODO 3 Empty Hopper Again
        // , new AcquisitionPivotCommand(AcquisitionSetpoint.RAISED),
        // followPathCommand(Paths.getDepotNeutralToShootingPath()),
        // new AutoShootCommand().withDeadline(new WaitCommand(2))
        );
    }
}
