package frc.robot.autons;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.PivotConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

public class MiddleAutoShoot extends Auton {
    public MiddleAutoShoot(Optional<DriverStation.Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddle1Path()),
                new PivotCommand(PivotConstants.PivotSetpoint.LOWERED),
                new AdjustHoodsCommand().raceWith(
                        new AlignCommand()
                                .andThen(new ShootCommand().withTimeout(5))));
    }
}
