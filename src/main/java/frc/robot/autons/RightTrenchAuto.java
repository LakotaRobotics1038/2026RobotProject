package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterCommand;

public class RightTrenchAuto extends Auton {
    public RightTrenchAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new HopperExtensionCommand(HopperExtensionCommand.ExtensionDirection.OUT),
                followPathCommand(Paths.getRight1Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE)),
                followPathCommand(Paths.getRight2Path()),
                new ShooterCommand().withTimeout(4).raceWith(
                        new ShootCommand()),
                followPathCommand(Paths.getRight3Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE)),
                followPathCommand(Paths.getRight4Path()),
                new AlignCommand().raceWith(
                        new ShootCommand(),
                        new ShooterCommand().withTimeout(4)));
    }
}
