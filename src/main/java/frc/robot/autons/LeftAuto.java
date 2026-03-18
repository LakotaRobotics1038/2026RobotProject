package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.AcquisitionRunCommand.Mode;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class LeftAuto extends Auton {
    public LeftAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new AcquisitionRunCommand(Mode.INTAKE),
                followPathCommand(Paths.getLeft1Path())
                        .alongWith(new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)));
        // super.addCommands(
        // followPathCommand(Paths.getMiddleToShootingPath()),
        // new AutoShootCommand().withTimeout(2),
        // followPathCommand(Paths.getShootingToDepotPath()).alongWith(
        // new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED)),
        // new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE)
        // .raceWith(followPathCommand(Paths.getAcquireFromDepotPath())),
        // followPathCommand(Paths.getDepotToShootingPath()),
        // new AutoShootCommand().withTimeout(2));
    }
}
