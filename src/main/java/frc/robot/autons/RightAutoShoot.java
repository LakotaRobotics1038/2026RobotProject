package frc.robot.autons;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;

import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

public class RightAutoShoot extends Auton {
    public RightAutoShoot(Optional<DriverStation.Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new AcquisitionPivotCommand(PivotSetpoint.LOWERED),
                followPathCommand(Paths.getRightShoot1Path()),
                new AdjustHoodsCommand().raceWith(
                        new AlignCommand()
                                .andThen(new ShootCommand().withTimeout(5))));
    }
}
