package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

public class CommandLogger {
    private static final ArrayList<Command> runningCommands = new ArrayList<>();

    public static void initialize() {
        CommandScheduler.getInstance().onCommandInitialize(command -> {
            runningCommands.add(command);
            logRunningCommands();
        });

        CommandScheduler.getInstance().onCommandInterrupt(command -> {
            runningCommands.remove(command);
            logRunningCommands();
        });

        CommandScheduler.getInstance().onCommandFinish(command -> {
            runningCommands.remove(command);
            logRunningCommands();
        });
    }

    private static void logRunningCommands() {
        String[] commands = new String[runningCommands.size()];
        for (int i = 0; i < runningCommands.size(); i++)
            commands[i] = runningCommands.get(i).getName();

        SmartDashboard.putStringArray("Running Commands", commands);
    }
}
