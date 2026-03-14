package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RepeatCommand extends SequentialCommandGroup
{
    public RepeatCommand(Command command, int amtTimes)
    {
        Command[] commands = new Command[amtTimes];
        for (int i = 0; i < amtTimes; i++)
        {
            commands[i] = command;
        }
        addCommands(commands);
    }
}
