package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionPivotTrenchRetract;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;

public class MiddleAuto extends Auton {
    public MiddleAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new AcquisitionPivotTrenchRetract(),
                followPathCommand(Paths.getMiddle1Path()),
                followPathCommand(Paths.getMiddle2Path()).raceWith(
                        new AcquisitionPivotCommand(PivotSetpoint.LOWERED)
                                .andThen(new AcquisitionCommand(AcquisitionCommand.Mode.INTAKE))),
                followPathCommand(Paths.getMiddle3Path()),
                new ShootCommand().withTimeout(4),
                followPathCommand(Paths.getMiddle4Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.Mode.INTAKE)),
                followPathCommand(Paths.getMiddle5Path()),
                new ShootCommand().withTimeout(4));
    }
}
