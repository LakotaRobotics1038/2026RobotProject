package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.HopperExtensionCommand.ExtensionDirection;
import frc.robot.commands.ShooterCommand;

public class LeftPreloadAuto extends Auton {
    public LeftPreloadAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new HopperExtensionCommand(ExtensionDirection.OUT),
                followPathCommand(Paths.getLeft1Path()),
                new AlignCommand().raceWith(new ShooterCommand().withTimeout(5)));
    }
}
