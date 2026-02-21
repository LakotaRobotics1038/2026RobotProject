package frc.robot.auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Stream;

public final class CommandValidator {
    private CommandValidator() {
    }

    public static void validateCommands() {
        Path pathPlannerDirectory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner");
        if (!Files.isDirectory(pathPlannerDirectory)) {
            String message = "PathPlanner directory not found: " + pathPlannerDirectory;
            DriverStation.reportError(message, false);
            return;
        }

        Set<String> registeredCommands = new HashSet<>(ValidatedNamedCommand.getCommands());

        try {
            Set<String> namedCommands = new HashSet<>();
            JSONParser parser = new JSONParser();

            try (Stream<Path> files = Files.walk(pathPlannerDirectory)) {
                files.filter(Files::isRegularFile)
                        .filter(path -> path.toString().endsWith(".auto"))
                        .forEach(path -> {
                            try (Reader reader = Files.newBufferedReader(path)) {
                                Object rawRoot = parser.parse(reader);
                                if (!(rawRoot instanceof JSONObject root)) {
                                    return;
                                }

                                Object rootCommand = root.get("command");
                                if (rootCommand instanceof JSONObject commandJson) {
                                    collectNamedCommands(commandJson, namedCommands);
                                }

                                Object eventMarkers = root.get("eventMarkers");
                                if (eventMarkers instanceof JSONArray markersArray) {
                                    for (Object markerObject : markersArray) {
                                        if (!(markerObject instanceof JSONObject markerJson)) {
                                            continue;
                                        }
                                        Object markerCommand = markerJson.get("command");
                                        if (markerCommand instanceof JSONObject markerCommandJson) {
                                            collectNamedCommands(markerCommandJson, namedCommands);
                                        }
                                    }
                                }
                            } catch (IOException | ParseException e) {
                                String message = "Skipping invalid PathPlanner file " + path + ": " + e.getMessage();
                                DriverStation.reportWarning(message, false);
                            }
                        });
            }

            namedCommands.removeAll(registeredCommands);
            if (!namedCommands.isEmpty()) {
                String message = "PathPlanner named command(s) used in JSON but not registered: " + namedCommands;
                DriverStation.reportError(message, false);
            }
        } catch (IOException e) {
            String message = "Failed to validate PathPlanner named commands: " + e.getMessage();
            DriverStation.reportWarning(message, false);
        }
    }

    private static void collectNamedCommands(JSONObject commandJson, Set<String> namedCommands) {
        Object typeObject = commandJson.get("type");
        Object dataObject = commandJson.get("data");
        if (!(typeObject instanceof String type) || !(dataObject instanceof JSONObject dataJson)) {
            return;
        }

        if (type.equals("named")) {
            Object nameObject = dataJson.get("name");
            if (nameObject instanceof String name && !name.isBlank()) {
                namedCommands.add(name);
            }
            return;
        }

        if (type.equals("sequential") || type.equals("parallel")
                || type.equals("deadline") || type.equals("race")) {
            Object commandsObject = dataJson.get("commands");
            if (commandsObject instanceof JSONArray commandsArray) {
                for (Object childObject : commandsArray) {
                    if (childObject instanceof JSONObject childCommandJson) {
                        collectNamedCommands(childCommandJson, namedCommands);
                    }
                }
            }
        }
    }
}
