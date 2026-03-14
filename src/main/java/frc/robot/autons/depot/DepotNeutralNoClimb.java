package frc.robot.autons.depot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class DepotNeutralNoClimb extends DepotNeutral {
    public DepotNeutralNoClimb(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(

        // TODO 4 acquire from neutral again ig
        // followPathCommand(Paths.getDepotShootingToNeutralPath()),
        // new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)
        // .andThen(new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE)
        // .raceWith(followPathCommand(Paths.getDepotAcquireFromNeutralPath())
        // .andThen(new WaitCommand(0.5)))),
        // new AcquisitionPivotCommand(AcquisitionSetpoint.RAISED)

        // TODO 5 shooting again
        // , followPathCommand(Paths.getDepotNeutralToShootingPath()),
        // new AutoShootCommand().withDeadline(new WaitCommand(2))
        );

    }
}
