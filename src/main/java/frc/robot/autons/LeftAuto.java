package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionPivotTrenchRetract;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.AcquisitionCommand.Mode;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;

public class LeftAuto extends Auton {
    public LeftAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new AcquisitionPivotTrenchRetract(),
                followPathCommand(Paths.getLeft1Path())
                        .raceWith(new AcquisitionPivotCommand(PivotSetpoint.LOWERED)
                                .andThen(new AcquisitionCommand(Mode.INTAKE))),
                followPathCommand(Paths.getLeft2Path()),
                new ShootCommand().withTimeout(4),
                followPathCommand(Paths.getLeft3Path())
                        .raceWith(new AcquisitionCommand(Mode.INTAKE)),
                followPathCommand(Paths.getLeft4Path()),
                new ShootCommand().withTimeout(4));
    }
}
