package frc.robot.auton;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.List;


public class ValidatedNamedCommand {
    private static final List<String> commands = new ArrayList<>();


    public static void registerCommand(Command command) {
        String commandName = command.getName();
        commands.add(commandName);
        NamedCommands.registerCommand(commandName, command);
    }

    public static List<String> getCommands() {
        return commands;
    }
}
