package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.HopperExtensionCommand.ExtensionDirection;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeDirection;
import frc.robot.commands.ShootCommand;

public class LeftAuto extends Auton {
    public LeftAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new HopperExtensionCommand(ExtensionDirection.FORWARD),
                followPathCommand(Paths.getLeftStartPath())
                        .raceWith(new IntakeCommand(IntakeDirection.INTAKE)),
                followPathCommand(Paths.getMiddleAcquireToShootPath()),
                new AlignCommand(() -> 0, () -> 0, null),
                new ShootCommand().withTimeout(5));
    }
}
