package frc.robot.autons;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class Paths {
    public static PathPlannerPath getMiddleToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle To Shooting");
    }

    public static PathPlannerPath getShootingToDepotPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Shooting To Depot");
    }

    public static PathPlannerPath getAcquireFromDepotPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot to Acquire depot");
    }

    public static PathPlannerPath getDepotToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot To Shooting");
    }

    public static PathPlannerPath getOutpostStartToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Start To Shooting)");
    }

    public static PathPlannerPath getOutpostShootingToNeutralPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Shooting To Neutral)");
    }

    public static PathPlannerPath getOutpostNeutralToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost(Neutral To Shooting)");
    }

    public static PathPlannerPath getDepotStartToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Start To Shooting)");
    }

    public static PathPlannerPath getDepotShootingToNeutralPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Shooting To Neutral)");
    }

    public static PathPlannerPath getDepotNeutralToAcquirePath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot(Neutral To Acquire)");
    }
}