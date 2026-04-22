package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AdjustHoodCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.ShootCommand;

public class DepotAuto extends Auton {
    public DepotAuto(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddleSideDepot1Path())
                        .alongWith(new HopperExtensionCommand(HopperExtensionCommand.ExtensionDirection.BACKWARD)),
                followPathCommand(Paths.getMiddleSideDepot2Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE)),
                followPathCommand(Paths.getMiddleSideDepot3Path()),
                new AlignCommand().raceWith(new AdjustHoodCommand().raceWith(
                        new ShootCommand().withTimeout(10))));
    }
}
