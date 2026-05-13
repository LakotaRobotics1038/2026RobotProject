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

public class FollowRightTrenchAuto extends Auton {
    public FollowRightTrenchAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getPreloadRight1Path()),
                new ShooterCommand().raceWith(new WaitCommand(2).andThen(new ShootCommand().withTimeout(2))),
                followPathCommand(Paths.getPreloadRight2Path()),
                followPathCommand(Paths.getRight1Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE))
                        .alongWith(new HopperExtensionCommand(HopperExtensionCommand.ExtensionDirection.OUT)),
                followPathCommand(Paths.getRight2Path()),
                new ShooterCommand().raceWith(new WaitCommand(2).andThen(new ShootCommand().withTimeout(2))),
                followPathCommand(Paths.getRight3Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.IntakeDirection.INTAKE))
                        .alongWith(new HopperExtensionCommand(HopperExtensionCommand.ExtensionDirection.OUT)),
                followPathCommand(Paths.getRight4Path()),
                new AlignCommand().raceWith(
                        new ShooterCommand(),
                        new WaitCommand(2).andThen(
                                new ShootCommand().withTimeout(2))));
    }
}
