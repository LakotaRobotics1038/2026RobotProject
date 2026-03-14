package frc.robot.autons.outpost;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.ClimbCommand;
import frc.robot.constants.ClimbConstants.ClimbSetpoint;

public class OutpostNeutralClimb extends OutpostNeutral {
    public OutpostNeutralClimb(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
        // followPathCommand(Paths.getOutpostShootingToClimb()),
        // new ClimbCommand(ClimbSetpoint.UP),
        // followPathCommand(Paths.getOutpostClimb()),
        // new ClimbCommand(ClimbSetpoint.DOWN)
        );
        /**
         * TODO 4 climb
         */
    }
}
