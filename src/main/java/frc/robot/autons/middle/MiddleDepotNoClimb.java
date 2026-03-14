package frc.robot.autons.middle;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MiddleDepotNoClimb extends MiddleDepot {
    public MiddleDepotNoClimb(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
        // TODO Collect more balls from somewhere and shoot. IDK where yet
        );
    }
}
