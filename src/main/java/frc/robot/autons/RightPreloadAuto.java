package frc.robot.autons;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.HopperExtensionCommand.ExtensionDirection;
import frc.robot.commands.ShooterCommand;

import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

public class RightPreloadAuto extends Auton {
    public RightPreloadAuto(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new HopperExtensionCommand(ExtensionDirection.IN),
                followPathCommand(Paths.getRight1Path()),
                new AlignCommand().raceWith(
                        new ShooterCommand(),
                        new WaitCommand(3).andThen(
                                new ShootCommand().withTimeout(10))));
    }
}
