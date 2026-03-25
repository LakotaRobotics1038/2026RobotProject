package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.AcquisitionRunCommand.Mode;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class LeftAuto extends Auton {
    public LeftAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getLeft1Path())
                        .raceWith(new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED).andThen(new AcquisitionRunCommand(Mode.INTAKE))),
                followPathCommand(Paths.getLeft2Path()),
                new ShootCommand().withTimeout(2),
                followPathCommand(Paths.getLeft3Path()).raceWith(new AcquisitionRunCommand(Mode.INTAKE)),
                followPathCommand(Paths.getLeft4Path()),
                new ShootCommand().withTimeout(2));
    }
}
