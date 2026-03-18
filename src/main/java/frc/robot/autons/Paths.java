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
}