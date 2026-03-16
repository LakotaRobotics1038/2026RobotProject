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

public class MiddleDepot extends Auton {
    public MiddleDepot(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddleToShootingPath()),
                new AutoShootCommand().withTimeout(2),
                followPathCommand(Paths.getShootingToDepotPath()).alongWith(
                        new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)),
                new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE)
                        .raceWith(followPathCommand(Paths.getAcquireFromDepotPath())),
                followPathCommand(Paths.getDepotToShootingPath()),
                new AutoShootCommand().withTimeout(2));
    }
}
