package frc.robot.lobby.subsystems.nfrdashboard;

import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
    private Map<String, DoubleSupplier> namesToDoubles = new HashMap<String, DoubleSupplier>();
    private Map<String, Supplier<String>> namesToStrings = new HashMap<String, Supplier<String>>();
    private Map<String, BooleanSupplier> namesToBooleans = new HashMap<String, BooleanSupplier>();
    private Map<String, Consumer<Double>> namesToDoubleTunables = new HashMap<String, Consumer<Double>>();
    private Map<String, Consumer<String>> namesToStringsTunables = new HashMap<String, Consumer<String>>();
    private Map<String, Consumer<Boolean>> namesToBooleansTunables = new HashMap<String, Consumer<Boolean>>();
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

    public void putNumber(String name, DoubleSupplier number)
    {
        if (!namesToDoubles.containsKey(name))
        {
            namesToDoubles.put(name, number);

            var numberTable = instance.getTable(outputPath).getSubTable("numbers").getSubTable(name);
            numberTable.getEntry("value").setDouble(number.getAsDouble());
        }
    }

    public void putString(String name, Supplier<String> string)
    {
        if (!namesToStrings.containsKey(name))
        {
            namesToStrings.put(name, string);

            var stringTable = instance.getTable(outputPath).getSubTable("strings").getSubTable(name);
            stringTable.getEntry("value").setString(string.get());
        }
    }

    public void putBoolean(String name, BooleanSupplier booleanSupplier)
    {
        if (!namesToBooleans.containsKey(name))
        {
            namesToBooleans.put(name, booleanSupplier);

            var booleanTable = instance.getTable(outputPath).getSubTable("booleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(booleanSupplier.getAsBoolean());
        }
    }

    public void putNumberTunable(String name, Consumer<Double> runOnChange)
    {
        if (!namesToDoubleTunables.containsKey(name))
        {

            namesToDoubleTunables.put(name, runOnChange);
            var doubleTable = instance.getTable(outputPath).getSubTable("tunableNumbers").getSubTable(name);
            doubleTable.getEntry("value").setNumber(0);
            doubleTable.getEntry("changed").setBoolean(false);

        }
    }

    public void putStringTunable(String name, Consumer<String> runOnChange)
    {
        if (!namesToStringsTunables.containsKey(name))
        {

            namesToStringsTunables.put(name, runOnChange);
            var stringTable = instance.getTable(outputPath).getSubTable("tunableStrings").getSubTable(name);
            stringTable.getEntry("value").setString("");
            stringTable.getEntry("changed").setBoolean(false);

        }
    }

    public void putBooleanTunable(String name, Consumer<Boolean> runOnChange)
    {
        if (!namesToBooleansTunables.containsKey(name))
        {

            namesToBooleansTunables.put(name, runOnChange);
            var booleanTable = instance.getTable(outputPath).getSubTable("tunableBooleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(false);
            booleanTable.getEntry("changed").setBoolean(false);

        }
    }

    @Override
    public void periodic()
    {
        Set<String> commandNames = namesToCommands.keySet();
        for (String name : commandNames)
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

        for (String name : namesToDoubles.keySet())
        {
            var numberTable = instance.getTable(outputPath).getSubTable("numbers").getSubTable(name);
            numberTable.getEntry("value").setDouble(namesToDoubles.get(name).getAsDouble());
        }
        for (String name : namesToStrings.keySet())
        {
            var stringTable = instance.getTable(outputPath).getSubTable("strings").getSubTable(name);
            stringTable.getEntry("value").setString(namesToStrings.get(name).get());
        }
        for (String name : namesToBooleans.keySet())
        {
            var booleanTable = instance.getTable(outputPath).getSubTable("booleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(namesToBooleans.get(name).getAsBoolean());
        }

        for (String name : namesToDoubleTunables.keySet())
        {
            var table = instance.getTable(outputPath).getSubTable("tunableNumbers").getSubTable(name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToDoubleTunables.get(name).accept(table.getEntry("value").getDouble(0));
            }
        }

        for (String name : namesToStringsTunables.keySet())
        {
            var table = instance.getTable(outputPath).getSubTable("tunableStrings").getSubTable(name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToStringsTunables.get(name).accept(table.getEntry("value").getString(""));
            }
        }

        for (String name : namesToBooleansTunables.keySet())
        {
            var table = instance.getTable(outputPath).getSubTable("tunableBooleans").getSubTable(name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToBooleansTunables.get(name).accept(table.getEntry("value").getBoolean(false));
            }
        }
    }
}
