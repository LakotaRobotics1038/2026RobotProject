package frc.robot.autons.middle;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autons.Auton;
import frc.robot.autons.Paths;
import frc.robot.commands.AcquisitionPivotCommand;
import frc.robot.commands.AcquisitionRunCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;

public class MiddleDepot extends Auton {
    public MiddleDepot(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                followPathCommand(Paths.getMiddleToShootingPath()),
                new AutoShootCommand().withDeadline(new WaitCommand(2))
        // TODO Acquire Depot
        // , followPathCommand(Paths.getShootingToDepotPath()),
        // new AcquisitionPivotCommand(AcquisitionSetpoint.LOWERED),
        // new AcquisitionRunCommand(AcquisitionRunCommand.Mode.INTAKE)
        // .raceWith(followPathCommand(Paths.getAcquireFromDepotPath()))
        // .andThen(new WaitCommand(1)),
        // new AcquisitionPivotCommand(AcquisitionSetpoint.RAISED),
        //
        // TODO Shoot
        // followPathCommand(Paths.getDepotToShootingPath()),
        // new AutoShootCommand().raceWith(new WaitCommand(2))
        );
    }
}
