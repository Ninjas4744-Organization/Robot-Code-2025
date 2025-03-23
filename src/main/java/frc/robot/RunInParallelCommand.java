package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RunInParallelCommand extends InstantCommand {
    public RunInParallelCommand(Command command) {
        super(command::schedule);
    }
}
