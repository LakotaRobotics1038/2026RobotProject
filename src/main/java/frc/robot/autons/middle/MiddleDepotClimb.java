package frc.robot.autons.middle;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.constants.ClimbConstants.ClimbSetpoint;

public class MiddleDepotClimb extends MiddleDepot {
    public MiddleDepotClimb(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
        // TODO climb
        // followPathCommand(Paths.getShootingToPrepClimb()),
        // new ClimbCommand(ClimbSetpoint.UP),
        // followPathCommand(Paths.getPrepClimbToClimb()),
        // new ClimbCommand(ClimbSetpoint.DOWN)
        );
    }
}
