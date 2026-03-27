package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.AcquisitionRunCommand.Mode;
import frc.robot.commands.AcquisitionTrenchRetract;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class RightAuto extends Auton {
    public RightAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new AcquisitionTrenchRetract(),
                followPathCommand(Paths.getRight1Path())
                /*
                 * .raceWith(new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)
                 * .andThen(new AcquisitionRunCommand(Mode.INTAKE)))
                 */,
                followPathCommand(Paths.getRight2Path()),
                new ShootCommand().withTimeout(4),
                followPathCommand(Paths.getRight3Path())/* .raceWith(new AcquisitionRunCommand(Mode.INTAKE)) */,
                followPathCommand(Paths.getRight4Path()),
                new ShootCommand().withTimeout(4));
    }
}
