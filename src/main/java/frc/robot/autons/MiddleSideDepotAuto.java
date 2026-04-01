package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AcquisitionCommand.Mode;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.AcquisitionPivotConstants;

public class MiddleSideDepotAuto extends Auton {
    public MiddleSideDepotAuto(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddleSideDepot1Path())
                        .alongWith(new AcquisitionPivotCommand(AcquisitionPivotConstants.PivotSetpoint.LOWERED)),
                followPathCommand(Paths.getMiddleSideDepot2Path())
                        .raceWith(new AcquisitionCommand(Mode.INTAKE)),
                followPathCommand(Paths.getMiddleSideDepot3Path()),
                new AdjustHoodsCommand().raceWith(
                        new ShootCommand().withTimeout(10)));
    }
}
