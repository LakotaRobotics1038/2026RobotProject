package frc.robot.autons;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.HopperExtensionCommand.ExtensionDirection;
import frc.robot.commands.ShootCommand;

import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

public class MiddlePreloadAuto extends Auton {
    public MiddlePreloadAuto(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddle1Path()),
                new HopperExtensionCommand(ExtensionDirection.OUT),
                new AlignCommand()
                        .raceWith(new ShootCommand().withTimeout(5)));
    }
}
