package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AcquisitionCommand;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AdjustHoodsCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.AcquisitionPivotConstants.PivotSetpoint;

public class RightAuto extends Auton {
    public RightAuto(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getRight1Path())
                        .alongWith(new AcquisitionPivotCommand(PivotSetpoint.LOWERED)
                                .andThen(new AcquisitionCommand(AcquisitionCommand.Mode.INTAKE).withTimeout(5))),
                followPathCommand(Paths.getRight2Path()),
                new AdjustHoodsCommand().raceWith(new ShootCommand().withTimeout(4)),
                new WaitCommand(2),
                followPathCommand(Paths.getRight3Path())
                        .alongWith(new AcquisitionCommand(AcquisitionCommand.Mode.INTAKE).withTimeout(5)),
                followPathCommand(Paths.getRight4Path()),
                new AdjustHoodsCommand().raceWith(new ShootCommand().withTimeout(4)));
    }
}
