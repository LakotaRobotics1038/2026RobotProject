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

    public static PathPlannerPath getOutpostStartToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Start To Shooting)");
    }

    public static PathPlannerPath getOutpostShootingToNeutralPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Shooting To Neutral)");
    }

    public static PathPlannerPath getOutpostAcquireFromNeutralPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Acquire From Neutral)");
    }

    public static PathPlannerPath getOutpostNeutralToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Neutral To Shooting)");
    }

    public static PathPlannerPath getOutpostShootingToClimbPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Shooting To PrepClimb)");
    }

    public static PathPlannerPath getOutpostClimbPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(PrepClimb To Climb)");
    }

    public static PathPlannerPath getDepotStartToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Start To Shooting)");
    }

    public static PathPlannerPath getDepotShootingToNeutralPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Shooting To Neutral)");
    }

    public static PathPlannerPath getDepotAcquireFromNeutralPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Acquire From Neutral)");
    }

    public static PathPlannerPath getDepotNeutralToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Neutral To Shooting)");
    }

    public static PathPlannerPath getDepotShootingToPrepClimbPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Shooting To Prep Climb)");
    }

    public static PathPlannerPath getDepotClimbPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Prep Climb To Climb)");
    }

}