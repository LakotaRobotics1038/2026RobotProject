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
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class RightAuto extends Auton {
    public RightAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getRight1Path())
                        .alongWith(new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED),
                                new AcquisitionRunCommand(Mode.INTAKE)),
                followPathCommand(Paths.getRight2Path()).alongWith(new AcquisitionRunCommand(Mode.STOP)),
                new ShootCommand().withTimeout(2),
                followPathCommand(Paths.getRight3Path()).alongWith(new AcquisitionRunCommand(Mode.INTAKE)),
                followPathCommand(Paths.getRight4Path()).alongWith(new AcquisitionRunCommand(Mode.STOP)),
                new ShootCommand().withTimeout(2));
    }
}
