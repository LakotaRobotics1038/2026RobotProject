package frc.robot.autons;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class Paths {
    public static PathPlannerPath getLeftStartPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Left Start to Middle Acquire");
    }

    public static PathPlannerPath getMiddleAcquireToShootPath()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle to Shoot");
    }

    public static PathPlannerPath getLeft1Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Left 1");
    }

    public static PathPlannerPath getMiddle1Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle 1");
    }

    public static PathPlannerPath getRight1Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Right 1");
    }

    public static PathPlannerPath getDepotLeft1Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot Left 1");
    }

    public static PathPlannerPath getDepotLeft2Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Depot Left 2");
    }
}