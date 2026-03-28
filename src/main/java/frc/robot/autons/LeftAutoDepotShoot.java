package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.HopperExtensionCommand;
import frc.robot.commands.HopperExtensionCommand.ExtensionDirection;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeDirection;
import frc.robot.commands.ShootCommand;

public class LeftAutoDepotShoot extends Auton {
    public LeftAutoDepotShoot(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getDepotLeft1Path())
                        .raceWith(new HopperExtensionCommand(ExtensionDirection.FORWARD)
                                .andThen(new IntakeCommand(IntakeDirection.INTAKE))),
                followPathCommand(Paths.getDepotLeft2Path()),
                new AdjustHoodsCommand().raceWith(
                        new AlignCommand()
                                .andThen(new ShootCommand().withTimeout(5))));
    }
}
