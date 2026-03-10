package frc.robot.autons;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class Paths {
    public static PathPlannerPath getLeftPosTaxiPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot Taxi");
    }

    public static PathPlannerPath getRightPosTaxiPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Outpost Taxi");
    }

    public static PathPlannerPath getMiddleToShootingPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle To Shooting");
    }
}