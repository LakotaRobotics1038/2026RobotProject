package frc.robot.autons;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class Paths {
    public static PathPlannerPath getLeft1Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Left 1");
    }

    public static PathPlannerPath getLeft2Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Left 2");
    }

    public static PathPlannerPath getLeft3Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Left 3");
    }

    public static PathPlannerPath getLeft4Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Left 4");
    }

    public static PathPlannerPath getMiddle1Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle 1");
    }

    public static PathPlannerPath getMiddle2Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle 2");
    }

    public static PathPlannerPath getMiddle3Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle 3");
    }

    public static PathPlannerPath getMiddle4Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle 4");
    }

    public static PathPlannerPath getMiddle5Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Middle 5");
    }

    public static PathPlannerPath getRight1Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Right 1");
    }

    public static PathPlannerPath getRight2Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Right 2");
    }

    public static PathPlannerPath getRight3Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Right 3");
    }

    public static PathPlannerPath getRight4Path()
            throws IOException, ParseException, FileVersionException {
        return PathPlannerPath.fromPathFile("Right 4");
    }
}