package frc.robot.lobby.subsystems.nfrdashboard;

import java.util.Map;
import java.util.HashMap;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase
{
    private String outputPath;
    private Map<String, Command> namesToCommands = new HashMap<String, Command>();
    private Map<String, Long> namesToLastRequestId = new HashMap<String, Long>();
    private Map<String, Command> namesToSafeCommands = new HashMap<String, Command>();
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
    // public Dashboard(String outputPath) {
    // this.outputPath = outputPath;
    // }

    public Dashboard()
    {
        outputPath = "/NFRDashboard";
    }

    public boolean putCommand(String name, Command command)
    {
        if (!namesToCommands.containsKey(name))
        {
            namesToCommands.put(name, command);
            namesToLastRequestId.put(name, 0L);

            var commandTable = instance.getTable(outputPath).getSubTable("commands").getSubTable(name);
            commandTable.getEntry("running").setBoolean(false);
            commandTable.getEntry("requested").setBoolean(false);
            commandTable.getEntry("requestId").setInteger(0);
            commandTable.getEntry("lastHandledRequestId").setInteger(0);

            Command safeCommand = new Command()
            {
                final Command inner = command;

                @Override
                public void initialize()
                {
                    var commandTable = instance.getTable(outputPath).getSubTable("commands").getSubTable(name);
                    commandTable.getEntry("requested").setBoolean(false);
                    commandTable.getEntry("running").setBoolean(true);
                    CommandScheduler.getInstance().schedule(inner);
                }

                @Override
                public void execute()
                {
                }

                @Override
                public void end(boolean interrupted)
                {
                    if (CommandScheduler.getInstance().isScheduled(inner))
                    {
                        CommandScheduler.getInstance().cancel(inner);
                    }
                    instance.getTable(outputPath).getSubTable("commands").getSubTable(name).getEntry("running")
                            .setBoolean(false);
                }

                @Override
                public boolean isFinished()
                {
                    return !CommandScheduler.getInstance().isScheduled(inner);
                }
            };

            namesToSafeCommands.put(name, safeCommand);

            return true;
        }
        return false;
    }

    @Override
    public void periodic()
    {
        Set<String> names = namesToCommands.keySet();
        for (String name : names)
        {
            var table = instance.getTable(outputPath).getSubTable("commands").getSubTable(name);
            long currentRequestId = table.getEntry("requestId").getInteger(0);
            long lastHandledRequestId = namesToLastRequestId.getOrDefault(name, 0L);

            if (currentRequestId != lastHandledRequestId)
            {
                namesToLastRequestId.put(name, currentRequestId);
                table.getEntry("lastHandledRequestId").setInteger(currentRequestId);
                table.getEntry("requested").setBoolean(false);

                Command safeCommand = namesToSafeCommands.get(name);
                if (safeCommand != null)
                {
                    if (CommandScheduler.getInstance().isScheduled(safeCommand))
                    {
                        CommandScheduler.getInstance().cancel(safeCommand);
                    } else
                    {
                        CommandScheduler.getInstance().schedule(safeCommand);
                    }
                }
            }
        }
    }
}
