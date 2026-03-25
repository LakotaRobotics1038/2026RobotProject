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

public class MiddleAuto extends Auton {
    public MiddleAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddle1Path()),
                followPathCommand(Paths.getMiddle2Path()).alongWith(
                        new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)
                                .andThen(new AcquisitionRunCommand(Mode.INTAKE).withTimeout(2))),
                followPathCommand(Paths.getMiddle3Path()),
                new ShootCommand().withTimeout(2),
                followPathCommand(Paths.getMiddle4Path()).raceWith(new AcquisitionRunCommand(Mode.INTAKE)),
                followPathCommand(Paths.getMiddle5Path()),
                new ShootCommand().withTimeout(2));
    }
}
