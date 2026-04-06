package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;

public class LeftTrenchAuto extends Auton {
    public LeftTrenchAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new AcquisitionPivotCommand(PivotSetpoint.LOWERED),
                followPathCommand(Paths.getLeft1Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.Mode.INTAKE)),
                followPathCommand(Paths.getLeft2Path()),
                new AdjustHoodsCommand().raceWith(new ShootCommand().withTimeout(4)),
                followPathCommand(Paths.getLeft3Path())
                        .raceWith(new AcquisitionCommand(AcquisitionCommand.Mode.INTAKE)),
                followPathCommand(Paths.getLeft4Path()),
                new AlignCommand().raceWith(new AdjustHoodsCommand().raceWith(new ShootCommand().withTimeout(4))));
    }
}