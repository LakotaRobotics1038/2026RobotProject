package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.HubAlignCommand;
import frc.robot.commands.ShootCommand;

public class SimpleLeftAuto extends Auton {
    public SimpleLeftAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getLeft1Path()),
                new AdjustHoodsCommand().raceWith(
                        new HubAlignCommand(() -> 0, () -> 0, null),
                        new ShootCommand().withTimeout(5)));
    }
}
