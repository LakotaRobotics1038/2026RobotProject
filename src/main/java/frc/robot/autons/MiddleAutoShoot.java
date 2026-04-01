package frc.robot.autons;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.AcquisitionPivotConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

public class MiddleAutoShoot extends Auton {
    public MiddleAutoShoot(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddle1Path()),
                new AcquisitionPivotCommand(AcquisitionPivotConstants.PivotSetpoint.LOWERED),
                new AdjustHoodsCommand().raceWith(
                        new AlignCommand()
                                .andThen(new ShootCommand().withTimeout(5))));
    }
}
