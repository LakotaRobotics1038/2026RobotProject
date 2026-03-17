package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class OutpostNeutral extends Auton {
    public OutpostNeutral(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getDepotStartToShootingPath()),
                new AutoShootCommand().withTimeout(2),
                followPathCommand(Paths.getDepotShootingToNeutralPath()).alongWith(
                        new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)),
                new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE)
                        .raceWith(followPathCommand(Paths.getAcquireFromDepotPath())),
                followPathCommand(Paths.getDepotNeutralToAcquirePath()),
                new AutoShootCommand().withTimeout(2));
    }
}
