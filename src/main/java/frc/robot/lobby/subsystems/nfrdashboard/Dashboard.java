package frc.robot.lobby.subsystems.nfrdashboard;

import java.util.Map;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase
{
    private String outputPath;

    private Map<String, Command> namesToCommands = new HashMap<>();
    private Map<String, Long> namesToLastRequestId = new HashMap<>();
    private Map<String, Command> namesToSafeCommands = new HashMap<>();
    private Map<String, DoubleSupplier> namesToDoubles = new HashMap<>();
    private Map<String, Supplier<String>> namesToStrings = new HashMap<>();
    private Map<String, BooleanSupplier> namesToBooleans = new HashMap<>();
    private Map<String, Consumer<Double>> namesToDoubleTunables = new HashMap<>();
    private Map<String, Consumer<String>> namesToStringsTunables = new HashMap<>();
    private Map<String, Consumer<Boolean>> namesToBooleansTunables = new HashMap<>();

    private NetworkTableInstance instance = NetworkTableInstance.getDefault();

    private Set<String> systems = new HashSet<>();
    // public Dashboard(String outputPath) {
    // this.outputPath = outputPath;
    // }

    public Dashboard()
    {
        outputPath = "/NFRDashboard";
    }

    private String makeKey(String tab, String table, String section, String name)
    {
        return tab + "::" + table + "::" + section + "::" + name;
    }

    private static class ParsedKey
    {
        final String tab, table, section, name;

        ParsedKey(String tab, String table, String section, String name)
        {
            this.tab = tab;
            this.table = table;
            this.section = section;
            this.name = name;
        }
    }

    private ParsedKey parseKey(String key)
    {
        String[] parts = key.split("::", 4);
        if (parts.length != 4)
            return null;
        return new ParsedKey(parts[0], parts[1], parts[2], parts[3]);
    }

    public boolean putCommand(String tab, String name, Command command)
    {
        String key = makeKey(tab, outputPath, "commands", name);
        if (!namesToCommands.containsKey(key))
        {
            namesToCommands.put(key, command);
            namesToLastRequestId.put(key, 0L);

            var commandTable = instance.getTable(outputPath).getSubTable("commands").getSubTable(name);
            commandTable.getEntry("running").setBoolean(false);
            commandTable.getEntry("requested").setBoolean(false);
            commandTable.getEntry("requestId").setInteger(0);
            commandTable.getEntry("lastHandledRequestId").setInteger(0);
            commandTable.getEntry("tab").setString(tab);

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

            namesToSafeCommands.put(key, safeCommand);

            return true;
        }
        return false;
    }

    public boolean putCommand(String tab, String table, String name, Command command)
    {
        String key = makeKey(tab, table, "commands", name);
        if (!namesToCommands.containsKey(key))
        {
            namesToCommands.put(key, command);
            namesToLastRequestId.put(key, 0L);

            var commandTable = instance.getTable(table).getSubTable("commands").getSubTable(name);
            commandTable.getEntry("running").setBoolean(false);
            commandTable.getEntry("requested").setBoolean(false);
            commandTable.getEntry("requestId").setInteger(0);
            commandTable.getEntry("lastHandledRequestId").setInteger(0);
            commandTable.getEntry("tab").setString(tab);

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

    public void putLimelightStream(String tab, String llName)
    {
        instance.getTable(outputPath).getSubTable("cameraStreams").getSubTable(llName).getEntry("url")
                .setString("http://" + llName + ".local:5800");
        instance.getTable(outputPath).getSubTable("cameraStreams").getSubTable(llName).getEntry("tab").setString(tab);
    }

    public void putCameraStream(String tab, String name, String url)
    {
        instance.getTable(outputPath).getSubTable("cameraStreams").getSubTable(name).getEntry("url").setString(url);
        instance.getTable(outputPath).getSubTable("cameraStreams").getSubTable(name).getEntry("tab").setString(tab);
    }

    public void putNumber(String tab, String name, DoubleSupplier number)
    {
        String key = makeKey(tab, outputPath, "numbers", name);
        if (!namesToDoubles.containsKey(key))
        {
            namesToDoubles.put(key, number);

            var numberTable = instance.getTable(outputPath).getSubTable("numbers").getSubTable(name);
            numberTable.getEntry("value").setDouble(number.getAsDouble());
            numberTable.getEntry("tab").setString(tab);
        }
    }

    public void putNumber(String tab, String table, String name, DoubleSupplier number)
    {
        String key = makeKey(tab, table, "numbers", name);
        if (!namesToDoubles.containsKey(key))
        {
            namesToDoubles.put(key, number);
            var numberTable = instance.getTable(table).getSubTable("numbers").getSubTable(name);
            numberTable.getEntry("value").setDouble(number.getAsDouble());
            numberTable.getEntry("tab").setString(tab);
        }
    }

    public void putString(String tab, String name, Supplier<String> string)
    {
        String key = makeKey(tab, outputPath, "strings", name);
        if (!namesToStrings.containsKey(key))
        {
            namesToStrings.put(key, string);

            var stringTable = instance.getTable(outputPath).getSubTable("strings").getSubTable(name);
            stringTable.getEntry("value").setString(string.get());
            stringTable.getEntry("tab").setString(tab);
        }
    }

    public void putString(String tab, String table, String name, Supplier<String> string)
    {
        String key = makeKey(tab, table, "strings", name);
        if (!namesToStrings.containsKey(key))
        {
            namesToStrings.put(key, string);
            var stringTable = instance.getTable(table).getSubTable("strings").getSubTable(name);
            stringTable.getEntry("value").setString(string.get());
            stringTable.getEntry("tab").setString(tab);
        }
    }

    public void putBoolean(String tab, String name, BooleanSupplier booleanSupplier)
    {
        String key = makeKey(tab, outputPath, "booleans", name);
        if (!namesToBooleans.containsKey(key))
        {
            namesToBooleans.put(key, booleanSupplier);
            var booleanTable = instance.getTable(outputPath).getSubTable("booleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(booleanSupplier.getAsBoolean());
            booleanTable.getEntry("tab").setString(tab);
        }
    }

    public void putBoolean(String tab, String table, String name, BooleanSupplier booleanSupplier)
    {
        String key = makeKey(tab, table, "booleans", name);
        if (!namesToBooleans.containsKey(key))
        {
            namesToBooleans.put(key, booleanSupplier);
            var booleanTable = instance.getTable(table).getSubTable("booleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(booleanSupplier.getAsBoolean());
            booleanTable.getEntry("tab").setString(tab);
        }
    }

    public void putNumberTunable(String tab, String name, Consumer<Double> runOnChange)
    {
        String key = makeKey(tab, outputPath, "tunableNumbers", name);
        if (!namesToDoubleTunables.containsKey(key))
        {

            namesToDoubleTunables.put(key, runOnChange);
            var doubleTable = instance.getTable(outputPath).getSubTable("tunableNumbers").getSubTable(name);
            doubleTable.getEntry("value").setNumber(0);
            doubleTable.getEntry("changed").setBoolean(false);
            doubleTable.getEntry("tab").setString(tab);

        }
    }

    public void putNumberTunable(String tab, String table, String name, Consumer<Double> runOnChange)
    {
        String key = makeKey(tab, table, "tunableNumbers", name);
        if (!namesToDoubleTunables.containsKey(key))
        {

            namesToDoubleTunables.put(key, runOnChange);
            var doubleTable = instance.getTable(table).getSubTable("tunableNumbers").getSubTable(name);
            doubleTable.getEntry("value").setNumber(0);
            doubleTable.getEntry("changed").setBoolean(false);
            doubleTable.getEntry("tab").setString(tab);

        }
    }

    public void putStringTunable(String tab, String name, Consumer<String> runOnChange)
    {
        String key = makeKey(tab, outputPath, "tunableStrings", name);
        if (!namesToStringsTunables.containsKey(key))
        {

            namesToStringsTunables.put(key, runOnChange);
            var stringTable = instance.getTable(outputPath).getSubTable("tunableStrings").getSubTable(name);
            stringTable.getEntry("value").setString("");
            stringTable.getEntry("changed").setBoolean(false);
            stringTable.getEntry("tab").setString(tab);
        }
    }

    public void putStringTunable(String tab, String table, String name, Consumer<String> runOnChange)
    {
        String key = makeKey(tab, table, "tunableStrings", name);
        if (!namesToStringsTunables.containsKey(key))
        {

            namesToStringsTunables.put(key, runOnChange);
            var stringTable = instance.getTable(table).getSubTable("tunableStrings").getSubTable(name);
            stringTable.getEntry("value").setString("");
            stringTable.getEntry("changed").setBoolean(false);
            stringTable.getEntry("tab").setString(tab);
        }
    }

    public void putBooleanTunable(String tab, String name, Consumer<Boolean> runOnChange)
    {
        String key = makeKey(tab, outputPath, "tunableBooleans", name);
        if (!namesToBooleansTunables.containsKey(key))
        {

            namesToBooleansTunables.put(key, runOnChange);
            var booleanTable = instance.getTable(outputPath).getSubTable("tunableBooleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(false);
            booleanTable.getEntry("changed").setBoolean(false);
            booleanTable.getEntry("tab").setString(tab);
        }
    }

    public void putBooleanTunable(String tab, String table, String name, Consumer<Boolean> runOnChange)
    {
        String key = makeKey(tab, table, "tunableBooleans", name);
        if (!namesToBooleansTunables.containsKey(key))
        {

            namesToBooleansTunables.put(key, runOnChange);
            var booleanTable = instance.getTable(table).getSubTable("tunableBooleans").getSubTable(name);
            booleanTable.getEntry("value").setBoolean(false);
            booleanTable.getEntry("changed").setBoolean(false);
            booleanTable.getEntry("tab").setString(tab);
        }
    }

    @Override
    public void periodic()
    {
        for (String key : namesToCommands.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = instance.getTable(p.table).getSubTable("commands").getSubTable(p.name);

            long currentRequestId = table.getEntry("requestId").getInteger(0);
            long lastHandledRequestId = namesToLastRequestId.getOrDefault(key, 0L);
            if (currentRequestId != lastHandledRequestId)
            {
                namesToLastRequestId.put(key, currentRequestId);
                table.getEntry("lastHandledRequestId").setInteger(currentRequestId);
                table.getEntry("requested").setBoolean(false);

                Command safe = namesToSafeCommands.get(key);
                if (safe != null)
                {
                    if (CommandScheduler.getInstance().isScheduled(safe))
                        CommandScheduler.getInstance().cancel(safe);
                    else
                        CommandScheduler.getInstance().schedule(safe);
                }
            }
        }

        for (String key : namesToDoubles.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            instance.getTable(p.table).getSubTable("numbers").getSubTable(p.name).getEntry("value")
                    .setDouble(namesToDoubles.get(key).getAsDouble());
        }
        for (String key : namesToStrings.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            instance.getTable(p.table).getSubTable("strings").getSubTable(p.name).getEntry("value")
                    .setString(namesToStrings.get(key).get());
        }
        for (String key : namesToBooleans.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            instance.getTable(p.table).getSubTable("booleans").getSubTable(p.name).getEntry("value")
                    .setBoolean(namesToBooleans.get(key).getAsBoolean());
        }
        for (String key : namesToDoubleTunables.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = instance.getTable(p.table).getSubTable("tunableNumbers").getSubTable(p.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToDoubleTunables.get(key).accept(table.getEntry("value").getDouble(0));
                table.getEntry("changed").setBoolean(false);
            }
        }
        for (String key : namesToStringsTunables.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = instance.getTable(p.table).getSubTable("tunableStrings").getSubTable(p.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToStringsTunables.get(key).accept(table.getEntry("value").getString(""));
                table.getEntry("changed").setBoolean(false);
            }
        }
        for (String key : namesToBooleansTunables.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = instance.getTable(p.table).getSubTable("tunableBooleans").getSubTable(p.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToBooleansTunables.get(key).accept(table.getEntry("value").getBoolean(false));
                table.getEntry("changed").setBoolean(false);
            }
        }
    }

    // changed order to match usage: putSystem(tab, systemName)
    public DashboardSystem putSystem(String tab, String systemName)
    {
        systems.add(systemName);
        return new DashboardSystem(systemName, tab);
    }

    public class DashboardSystem
    {
        private String name;
        private String tab;

        public DashboardSystem(String name, String tab)
        {
            this.name = name;
            this.tab = tab;
        }

        public String getName()
        {
            return name;
        }

        public String getTab()
        {
            return tab;
        }

        public DashboardSystem withCommand(String name, Command command)
        {
            Dashboard.this.putCommand(tab, outputPath + "/systems/" + this.name, name, command);
            return this;
        }

        public DashboardSystem withNumber(String name, DoubleSupplier number)
        {
            Dashboard.this.putNumber(tab, outputPath + "/systems/" + this.name, name, number);
            return this;
        }

        public DashboardSystem withString(String name, Supplier<String> string)
        {
            Dashboard.this.putString(tab, outputPath + "/systems/" + this.name, name, string);
            return this;
        }

        public DashboardSystem withBoolean(String name, BooleanSupplier booleanSupplier)
        {
            Dashboard.this.putBoolean(tab, outputPath + "/systems/" + this.name, name, booleanSupplier);
            return this;
        }

        public DashboardSystem withNumberTunable(String name, Consumer<Double> runOnChange)
        {
            Dashboard.this.putNumberTunable(tab, outputPath + "/systems/" + this.name, name, runOnChange);
            return this;
        }

        public DashboardSystem withStringTunable(String name, Consumer<String> runOnChange)
        {
            Dashboard.this.putStringTunable(tab, outputPath + "/systems/" + this.name, name, runOnChange);
            return this;
        }

        public DashboardSystem withBooleanTunable(String name, Consumer<Boolean> runOnChange)
        {
            Dashboard.this.putBooleanTunable(tab, outputPath + "/systems/" + this.name, name, runOnChange);
            return this;

        }
    }
}
