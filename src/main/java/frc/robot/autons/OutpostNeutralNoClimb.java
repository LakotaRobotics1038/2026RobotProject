package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class OutpostNeutralNoClimb extends OutpostNeutral {
    public OutpostNeutralNoClimb(Optional<Alliance> alliance)
            throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
        /**
         * TODO 4 collect from some where
         * TODO 5 shoot from some where
         */
        );

    }
}
