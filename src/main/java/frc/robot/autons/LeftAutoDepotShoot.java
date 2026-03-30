package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.AcquisitionRunCommand.Mode;
import frc.robot.constants.PivotConstants.PivotSetpoint;

public class LeftAutoDepotShoot extends Auton {
    public LeftAutoDepotShoot(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getDepotLeft1Path())
                        .alongWith(new AcquisitionPivotCommand(PivotSetpoint.LOWERED)
                                .andThen(new AcquisitionRunCommand(Mode.INTAKE).withTimeout(5))),
                followPathCommand(Paths.getDepotLeft2Path()),
                new AdjustHoodsCommand().raceWith(
                        new AlignCommand()
                                .andThen(new ShootCommand().withTimeout(5))));
    }
}
