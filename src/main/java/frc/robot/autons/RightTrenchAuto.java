package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import frc.robot.commands.*;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RightTrenchAuto extends Auton {
    public RightTrenchAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new HopperExtensionCommand(HopperExtensionCommand.ExtensionDirection.BACKWARD),
                followPathCommand(Paths.getRight1Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE)),
                followPathCommand(Paths.getRight2Path()),
                new AdjustHoodCommand().raceWith(new ShootCommand().withTimeout(4)),
                followPathCommand(Paths.getRight3Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE)),
                followPathCommand(Paths.getRight4Path()),
                new AlignCommand().raceWith(new AdjustHoodCommand().raceWith(new ShootCommand().withTimeout(4))));
    }
}
