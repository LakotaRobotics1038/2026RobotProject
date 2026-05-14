package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterCommand;

public class FollowLeftTrenchAuto extends Auton {
    public FollowLeftTrenchAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getPreloadLeft1Path()).alongWith(new ShooterCommand()),
                new ShooterCommand().raceWith(new WaitCommand(2).andThen(new ShootCommand().withTimeout(2))),
                followPathCommand(Paths.getPreloadLeft2Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE))
                        .alongWith(new HopperExtensionCommand(HopperExtensionCommand.ExtensionDirection.OUT)),
                followPathCommand(Paths.getLeft2Path()),
                new ShooterCommand().raceWith(new WaitCommand(2).andThen(new ShootCommand().withTimeout(2))),
                followPathCommand(Paths.getLeft3Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE))
                        .alongWith(new HopperExtensionCommand(HopperExtensionCommand.ExtensionDirection.OUT)),
                followPathCommand(Paths.getLeft4Path()),
                new AlignCommand().raceWith(
                        new ShooterCommand(),
                        new WaitCommand(2).andThen(new ShootCommand().withTimeout(2))));
    }
}
