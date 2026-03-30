package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.PivotConstants.PivotSetpoint;

public class LeftAutoShoot extends Auton {
    public LeftAutoShoot(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new PivotCommand(PivotSetpoint.LOWERED),
                followPathCommand(Paths.getLeft1Path()),
                new AdjustHoodsCommand().raceWith(
                        new AlignCommand()
                                .andThen(new ShootCommand().withTimeout(5))));
    }
}
