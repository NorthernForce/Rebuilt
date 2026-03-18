package frc.robot.lobby.subsystems.nfrdashboard;

import java.util.Map;
import java.util.HashMap;
import java.util.HashSet;
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
import frc.robot.LimelightHelpers;

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

    private Set<String> systems = new HashSet<>();
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

    public boolean putCommand(String table, String name, Command command)
    {
        String key = table + "/commands/" + name;
        if (!namesToCommands.containsKey(key))
        {
            namesToCommands.put(key, command);
            namesToLastRequestId.put(key, 0L);

            var commandTable = instance.getTable(table).getSubTable("commands").getSubTable(name);
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
                    var commandTable = instance.getTable(table).getSubTable("commands").getSubTable(name);
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
                    instance.getTable(table).getSubTable("commands").getSubTable(name).getEntry("running")
                            .setBoolean(false);
                }

                @Override
                public boolean isFinished()
                {
                    return !CommandScheduler.getInstance().isScheduled(inner);
                }
            };

            namesToSafeCommands.put(key, safeCommand);

            return true;
        }
        return false;
    }

    public void putLimelightStream(String llName)
    {
        instance.getTable(outputPath).getSubTable("cameraStreams").getEntry(llName)
                .setString("https://" + llName + ".local:5800");
    }

    public void putCameraStream(String name, String url)
    {
        instance.getTable(outputPath).getSubTable("cameraStreams").getEntry(name).setString(url);
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

    public void putNumber(String table, String name, DoubleSupplier number)
    {
        String key = table + "/numbers/" + name;
        if (!namesToDoubles.containsKey(key))
        {
            namesToDoubles.put(key, number);
        }
        var numberTable = instance.getTable(table).getSubTable("numbers").getSubTable(name);
        numberTable.getEntry("value").setDouble(number.getAsDouble());
    }

    public void putString(String table, String name, Supplier<String> string)
    {
        String key = table + "/strings/" + name;
        if (!namesToStrings.containsKey(key))
        {
            namesToStrings.put(key, string);
        }
        var stringTable = instance.getTable(table).getSubTable("strings").getSubTable(name);
        stringTable.getEntry("value").setString(string.get());
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

    public void putString(String name, Supplier<String> string)
    {
        if (!namesToStrings.containsKey(name))
        {
            namesToStrings.put(name, string);

            var stringTable = instance.getTable(outputPath).getSubTable("strings").getSubTable(name);
            stringTable.getEntry("value").setString(string.get());
        }
    }

    public void putBoolean(String table, String name, BooleanSupplier booleanSupplier)
    {
        String key = table + "/booleans/" + name;
        if (!namesToBooleans.containsKey(key))
        {
            namesToBooleans.put(key, booleanSupplier);
        }
        var booleanTable = instance.getTable(table).getSubTable("booleans").getSubTable(name);
        booleanTable.getEntry("value").setBoolean(booleanSupplier.getAsBoolean());
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

    public void putNumberTunable(String table, String name, Consumer<Double> runOnChange)
    {
        String key = table + "/tunableNumbers/" + name;
        if (!namesToDoubleTunables.containsKey(key))
        {

            namesToDoubleTunables.put(key, runOnChange);
            var doubleTable = instance.getTable(table).getSubTable("tunableNumbers").getSubTable(name);
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

    public void putStringTunable(String table, String name, Consumer<String> runOnChange)
    {
        String key = table + "/tunableStrings/" + name;
        if (!namesToStringsTunables.containsKey(key))
        {

            namesToStringsTunables.put(key, runOnChange);
            var stringTable = instance.getTable(table).getSubTable("tunableStrings").getSubTable(name);
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

    public void putBooleanTunable(String table, String name, Consumer<Boolean> runOnChange)
    {
        String key = table + "/tunableBooleans/" + name;
        if (!namesToBooleansTunables.containsKey(key))
        {

            namesToBooleansTunables.put(key, runOnChange);
            var booleanTable = instance.getTable(table).getSubTable("tunableBooleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(false);
            booleanTable.getEntry("changed").setBoolean(false);

        }
    }

    private static class ParsedKey
    {
        final String table;
        final String name;

        ParsedKey(String table, String name)
        {
            this.table = table;
            this.name = name;
        }
    }

    private ParsedKey parseCompositeKey(String key, String section)
    {
        String marker = "/" + section + "/";
        int idx = key.indexOf(marker);
        if (idx < 0)
            return null;
        String table = key.substring(0, idx);
        String name = key.substring(idx + marker.length());
        return new ParsedKey(table, name);
    }

    @Override
    public void periodic()
    {
        Set<String> commandNames = namesToCommands.keySet();
        for (String key : commandNames)
        {
            ParsedKey parsed = parseCompositeKey(key, "commands");
            var table = (parsed == null) ? instance.getTable(outputPath).getSubTable("commands").getSubTable(key)
                    : instance.getTable(parsed.table).getSubTable("commands").getSubTable(parsed.name);

            long currentRequestId = table.getEntry("requestId").getInteger(0);
            long lastHandledRequestId = namesToLastRequestId.getOrDefault(key, 0L);

            if (currentRequestId != lastHandledRequestId)
            {
                namesToLastRequestId.put(key, currentRequestId);
                table.getEntry("lastHandledRequestId").setInteger(currentRequestId);
                table.getEntry("requested").setBoolean(false);

                Command safeCommand = namesToSafeCommands.get(key);
                if (safeCommand != null)
                {
                    if (CommandScheduler.getInstance().isScheduled(safeCommand))
                        CommandScheduler.getInstance().cancel(safeCommand);
                    else
                        CommandScheduler.getInstance().schedule(safeCommand);
                }
            }
        }

        for (String key : namesToDoubles.keySet())
        {
            ParsedKey parsed = parseCompositeKey(key, "numbers");
            if (parsed == null)
            {
                var numberTable = instance.getTable(outputPath).getSubTable("numbers").getSubTable(key);
                numberTable.getEntry("value").setDouble(namesToDoubles.get(key).getAsDouble());
            } else
            {
                var numberTable = instance.getTable(parsed.table).getSubTable("numbers").getSubTable(parsed.name);
                numberTable.getEntry("value").setDouble(namesToDoubles.get(key).getAsDouble());
            }
        }
        for (String key : namesToStrings.keySet())
        {
            ParsedKey parsed = parseCompositeKey(key, "strings");
            if (parsed == null)
            {
                var stringTable = instance.getTable(outputPath).getSubTable("strings").getSubTable(key);
                stringTable.getEntry("value").setString(namesToStrings.get(key).get());
            } else
            {
                var stringTable = instance.getTable(parsed.table).getSubTable("strings").getSubTable(parsed.name);
                stringTable.getEntry("value").setString(namesToStrings.get(key).get());
            }
        }
        for (String key : namesToBooleans.keySet())
        {
            ParsedKey parsed = parseCompositeKey(key, "booleans");
            if (parsed == null)
            {
                var booleanTable = instance.getTable(outputPath).getSubTable("booleans").getSubTable(key);
                booleanTable.getEntry("value").setBoolean(namesToBooleans.get(key).getAsBoolean());
            } else
            {
                var booleanTable = instance.getTable(parsed.table).getSubTable("booleans").getSubTable(parsed.name);
                booleanTable.getEntry("value").setBoolean(namesToBooleans.get(key).getAsBoolean());
            }
        }

        for (String key : namesToDoubleTunables.keySet())
        {
            ParsedKey parsed = parseCompositeKey(key, "tunableNumbers");
            var table = (parsed == null) ? instance.getTable(outputPath).getSubTable("tunableNumbers").getSubTable(key)
                    : instance.getTable(parsed.table).getSubTable("tunableNumbers").getSubTable(parsed.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToDoubleTunables.get(key).accept(table.getEntry("value").getDouble(0));
                table.getEntry("changed").setBoolean(false);
            }
        }

        for (String key : namesToStringsTunables.keySet())
        {
            ParsedKey parsed = parseCompositeKey(key, "tunableStrings");
            var table = (parsed == null) ? instance.getTable(outputPath).getSubTable("tunableStrings").getSubTable(key)
                    : instance.getTable(parsed.table).getSubTable("tunableStrings").getSubTable(parsed.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToStringsTunables.get(key).accept(table.getEntry("value").getString(""));
                table.getEntry("changed").setBoolean(false);
            }
        }

        for (String key : namesToBooleansTunables.keySet())
        {
            ParsedKey parsed = parseCompositeKey(key, "tunableBooleans");
            var table = (parsed == null) ? instance.getTable(outputPath).getSubTable("tunableBooleans").getSubTable(key)
                    : instance.getTable(parsed.table).getSubTable("tunableBooleans").getSubTable(parsed.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToBooleansTunables.get(key).accept(table.getEntry("value").getBoolean(false));
                table.getEntry("changed").setBoolean(false);
            }
        }
    }

    public DashboardSystem putSystem(String systemName)
    {
        systems.add(systemName);
        return new DashboardSystem(systemName);
    }

    public class DashboardSystem
    {
        private String name;

        public DashboardSystem(String name)
        {
            this.name = name;
        }

        public String getName()
        {
            return name;
        }

        public DashboardSystem withCommand(String name, Command command)
        {
            Dashboard.this.putCommand(outputPath + "/systems/" + this.name, name, command);
            return this;
        }

        public DashboardSystem withNumber(String name, DoubleSupplier number)
        {
            Dashboard.this.putNumber(outputPath + "/systems/" + this.name, name, number);
            return this;
        }

        public DashboardSystem withString(String name, Supplier<String> string)
        {
            Dashboard.this.putString(outputPath + "/systems/" + this.name, name, string);
            return this;
        }

        public DashboardSystem withBoolean(String name, BooleanSupplier booleanSupplier)
        {
            Dashboard.this.putBoolean(outputPath + "/systems/" + this.name, name, booleanSupplier);
            return this;
        }

        public DashboardSystem withNumberTunable(String name, Consumer<Double> runOnChange)
        {
            Dashboard.this.putNumberTunable(outputPath + "/systems/" + this.name, name, runOnChange);
            return this;
        }

        public DashboardSystem withStringTunable(String name, Consumer<String> runOnChange)
        {
            Dashboard.this.putStringTunable(outputPath + "/systems/" + this.name, name, runOnChange);
            return this;
        }

        public DashboardSystem withBooleanTunable(String name, Consumer<Boolean> runOnChange)
        {
            Dashboard.this.putBooleanTunable(outputPath + "/systems/" + this.name, name, runOnChange);
            return this;

        }
    }
}
