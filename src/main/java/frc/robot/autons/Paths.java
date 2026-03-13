package frc.robot.autons;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class Paths {
    public static PathPlannerPath getLeftPosTaxiPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("TopStartingPos Taxi");
    }

    public static PathPlannerPath getRightPosTaxiPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("BottomStartingPos Taxi");
    }

    public static PathPlannerPath getOutpostStartToShooting()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Start To Shooting)");
    }

    public static PathPlannerPath getOutpostShootingToNeutral()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Shooting To Neutral)");
    }

    public static PathPlannerPath getOutpostAcquireFromNeutral()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Acquire From Neutral)");
    }

    public static PathPlannerPath getOutpostNeutralToShooting()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Neutral To Shooting)");
    }

    public static PathPlannerPath getOutpostShootingToClimb()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Shooting To PrepClimb)");
    }

    public static PathPlannerPath getOutpostClimb()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(PrepClimb To Climb)");
    }
}