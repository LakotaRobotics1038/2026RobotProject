package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.PivotConstants.PivotSetpoint;

public class LeftAuto extends Auton {
    public LeftAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new PivotCommand(PivotSetpoint.LOWERED),
                followPathCommand(Paths.getLeftStartPath())
                        .raceWith(new IndexerCommand(IndexerCommand.Mode.INTAKE)),
                followPathCommand(Paths.getMiddleAcquireToShootPath()),
                new AlignCommand(() -> 0, () -> 0, null),
                new ShootCommand().withTimeout(5));
    }
}
